// On complete, checks CRC-32 over the d data bytes of SDRAM slots 0..n-1; otherwise serves the game CPU's slot reads
// (img_*), or re-reads the JTAG-selected slot every 10 us (tools/datatape.py check --dump).
module data_verify (
    input  wire         clk,
    input  wire         rst,
    input  wire         complete,       // from the 27 MHz loader; n, d and crc_exp change only as it drops
    input  wire [15:0]  n,
    input  wire [4:0]   d,
    input  wire [31:0]  crc_exp,
    input  wire [14:0]  peek_idx,       // from JTAG
    output reg          rd_req,         // pulse: read slot rd_idx
    output reg  [14:0]  rd_idx,
    input  wire         rv,             // its 16 words, in any order
    input  wire [3:0]   rcol,
    input  wire [15:0]  rdata,
    output reg          verified,
    output reg          verify_ok,
    output reg  [31:0]  crc_got,
    output reg  [14:0]  peek_at,
    output reg  [135:0] peek,           // byte 0 in [135:128]
    input  wire         img_req,        // pulse: read img_count slots from img_first
    input  wire [15:0]  img_first,
    input  wire [15:0]  img_count,
    output reg          img_ready,      // slot img_idx is loaded; img_byte is its byte img_sel
    output reg  [14:0]  img_idx,
    input  wire [4:0]   img_sel,
    output wire [7:0]   img_byte,
    input  wire         img_done        // pulse: the slot has been sent
);
    localparam [2:0] V_IDLE = 3'd0, V_REQ = 3'd1, V_WAIT = 3'd2, V_PROC = 3'd3, V_DONE = 3'd4, V_PEEK = 3'd5,
                     V_IMG = 3'd6;

    function [31:0] crc32_byte(input [31:0] c, input [7:0] b);
        integer k;
        reg [31:0] x;
        begin
            x = c ^ {24'd0, b};
            for (k = 0; k < 8; k = k + 1) x = x[0] ? (x >> 1) ^ 32'hEDB88320 : x >> 1;
            crc32_byte = x;
        end
    endfunction

    reg  [2:0]  state;
    reg  [2:0]  comp_s;
    reg  [14:0] peek_m, peek_s;
    reg         verifying;
    reg  [15:0] i;
    reg  [4:0]  j, got;
    reg  [15:0] slot [0:15];
    reg  [31:0] crc;
    reg  [13:0] settle;
    reg  [9:0]  ptimer;
    reg  [11:0] timeout;
    integer     k;

    wire [15:0] sw = slot[j[4:1]];
    wire [7:0]  b  = j[0] ? sw[15:8] : sw[7:0];
    wire [15:0] iw = slot[img_sel[4:1]];
    assign img_byte = img_sel[0] ? iw[15:8] : iw[7:0];

    reg         imaging;
    reg         img_stale;              // an img_req replaced the slot in flight: finish it, don't advance
    reg  [15:0] img_left;
    reg  [14:0] img_next;
    reg  [15:0] n_l;                    // n, d and crc_exp as verification started
    reg  [4:0]  d_l;
    reg  [31:0] crc_l;

    always @(posedge clk) begin
        comp_s <= {comp_s[1:0], complete};
        {peek_s, peek_m} <= {peek_m, peek_idx};
        rd_req <= 1'b0;
        if (img_req) begin
            img_next <= img_first[14:0];
            img_left <= img_count;
            if (imaging && (state == V_REQ || state == V_WAIT || state == V_IMG)) img_stale <= 1'b1;
        end
        if (rst) begin
            state     <= V_IDLE;
            verified  <= 1'b0;
            settle    <= 14'd0;
            ptimer    <= 10'd0;
            img_left  <= 16'd0;
            img_ready <= 1'b0;
            img_stale <= 1'b0;
        end else begin
            if (!comp_s[2]) verified <= 1'b0;
            case (state)
                V_IDLE: begin
                    img_stale <= 1'b0;
                    settle <= comp_s[2] ? settle + !(&settle) : 14'd0;
                    ptimer <= ptimer + 1'b1;
                    if (comp_s[2] && !verified) begin
                        if (&settle) begin  // 160 us: the last packet writes have landed
                            verifying <= 1'b1;
                            imaging   <= 1'b0;
                            i         <= 16'd0;
                            crc       <= 32'hFFFFFFFF;
                            n_l       <= n;
                            d_l       <= d;
                            crc_l     <= crc_exp;
                            state     <= V_REQ;
                        end
                    end else if (img_left != 16'd0 && !img_req) begin
                        verifying <= 1'b0;
                        imaging   <= 1'b1;
                        i         <= {1'b0, img_next};
                        state     <= V_REQ;
                    end else if (ptimer == 10'd999) begin
                        verifying <= 1'b0;
                        imaging   <= 1'b0;
                        i         <= {1'b0, peek_s};
                        ptimer    <= 10'd0;
                        state     <= V_REQ;
                    end
                end
                V_REQ: begin
                    rd_req  <= 1'b1;
                    rd_idx  <= i[14:0];
                    got     <= 5'd0;
                    timeout <= 12'd0;
                    state   <= V_WAIT;
                end
                V_WAIT: begin
                    timeout <= timeout + 1'b1;
                    if (rv) begin
                        slot[rcol] <= rdata;
                        got        <= got + 1'b1;
                        if (got == 5'd15) begin
                            j     <= 5'd0;
                            state <= verifying ? V_PROC : !imaging ? V_PEEK : img_stale ? V_IDLE : V_IMG;
                        end
                    end else if (&timeout) state <= V_IDLE;
                end
                V_PROC: begin
                    crc <= crc32_byte(crc, b);
                    j   <= j + 1'b1;
                    if (j == d_l - 1'b1) begin
                        i     <= i + 1'b1;
                        state <= !comp_s[2] ? V_IDLE : i + 1'b1 == n_l ? V_DONE : V_REQ;
                    end
                end
                V_DONE: begin
                    crc_got   <= ~crc;
                    verify_ok <= ~crc == crc_l;
                    verified  <= comp_s[2];
                    state     <= V_IDLE;
                end
                V_PEEK: begin
                    for (k = 0; k < 17; k = k + 1)
                        peek[135 - 8 * k -: 8] <= k[0] ? slot[k >> 1][15:8] : slot[k >> 1][7:0];
                    peek_at <= i[14:0];
                    state   <= V_IDLE;
                end
                V_IMG:                      // a replaced slot already offered still goes out once, intact
                    if (img_done) begin
                        img_ready <= 1'b0;
                        state     <= V_IDLE;
                        if (!img_stale && !img_req) begin
                            img_next <= img_next + 1'b1;
                            img_left <= img_left - 1'b1;
                        end
                    end else if (img_stale && !img_ready) state <= V_IDLE;
                    else begin
                        img_ready <= 1'b1;
                        img_idx   <= i[14:0];
                    end
                default: state <= V_IDLE;
            endcase
        end
    end
endmodule
