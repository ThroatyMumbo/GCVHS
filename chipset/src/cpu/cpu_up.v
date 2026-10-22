// UP packets out as nibbles: data, then a strobe edge 2 clocks later, held 2 more;
// low nibble on the rising edge, high on the falling, so the strobe idles low between bytes.
module cpu_up #(
    parameter integer SB = 64               // STATUS payload bytes
) (
    input  wire          clk,               // sd_clk
    input  wire          rst,
    input  wire          status_req,        // pulse: send status
    input  wire [8*SB-1:0] status,          // byte 0 in [7:0]; latched when the packet starts
    input  wire          aud_req,           // pulse: send aud_frames, the running total granted
    input  wire [15:0]   aud_frames,
    input  wire          echo_we,
    input  wire [7:0]    echo_waddr,
    input  wire [7:0]    echo_wdata,
    input  wire          echo_done,
    input  wire [8:0]    echo_len,
    input  wire          img_ready,         // a slot is loaded: img_idx, img_d, img_byte(img_sel)
    input  wire [14:0]   img_idx,
    input  wire [4:0]    img_d,
    output reg  [4:0]    img_sel,
    input  wire [7:0]    img_byte,
    output reg           img_done,          // pulse: the slot went out
    input  wire          tap_req,           // pulse: send tap_line; dropped while the last is still waiting
    input  wire [175:0]  tap_line,
    output reg  [15:0]   tap_drops,
    output reg           strobe,
    output reg  [3:0]    nib,
    output reg  [15:0]   packets
);
    `include "link_proto.vh"
    `include "crc16.vh"

    reg          st_pend, ar_pend, ec_pend, tp_pend;
    reg  [15:0]  ar_frames;
    reg  [8:0]   ec_len;
    reg  [8*SB-1:0] st_lat;
    reg  [175:0] tp_buf, tp_lat;

    // Builder: one byte ready ahead of the serializer. k counts bytes in the packet, of total;
    // k holds for the two fetch cycles, so reads addressed by it are ready in the second.
    reg          busy, nxt_v, fetch;
    reg  [7:0]   typ, nxt;
    reg  [15:0]  len, k, total, crc;
    wire [15:0]  p = k - 16'd4;             // payload index

    reg [7:0] echo_ram [0:255];
    reg [7:0] echo_rd;
    always @(posedge clk) begin
        if (echo_we) echo_ram[echo_waddr] <= echo_wdata;
        echo_rd <= echo_ram[p[7:0]];
    end
    reg  [7:0]  pay;
    always @* case (typ)
        UP_STATUS:   pay = st_lat[{p[5:0], 3'b0} +: 8];
        UP_AUD_REQ:  pay = p[0] ? ar_frames[15:8] : ar_frames[7:0];
        UP_ECHO:     pay = echo_rd;
        UP_TAPE_AUD: pay = tp_lat[{p[4:0], 3'b0} +: 8];
        default:   pay = p == 16'd0 ? img_idx[7:0] : p == 16'd1 ? {1'b0, img_idx[14:8]} : img_byte;
    endcase

    reg        take;                        // serializer took nxt this cycle
    always @(posedge clk) begin
        img_done <= 1'b0;
        if (status_req) st_pend <= 1'b1;
        if (aud_req) begin
            ar_pend   <= 1'b1;
            ar_frames <= aud_frames;
        end
        if (echo_done) begin
            ec_pend <= 1'b1;
            ec_len  <= echo_len;
        end
        if (tap_req) begin
            if (tp_pend) tap_drops <= tap_drops + 1'b1;
            else begin
                tp_pend <= 1'b1;
                tp_buf  <= tap_line;
            end
        end
        if (take) nxt_v <= 1'b0;
        if (rst) begin
            {st_pend, ar_pend, ec_pend, tp_pend, busy, nxt_v, fetch} <= 7'd0;
            packets   <= 16'd0;
            tap_drops <= 16'd0;
        end else if (!busy) begin
            k <= 16'd0;
            if (st_pend) begin
                busy <= 1'b1; typ <= UP_STATUS; len <= SB; st_lat <= status; st_pend <= 1'b0;
            end else if (ar_pend) begin
                busy <= 1'b1; typ <= UP_AUD_REQ; len <= UP_AUD_REQ_LEN; ar_pend <= 1'b0;
            end else if (ec_pend) begin
                busy <= 1'b1; typ <= UP_ECHO; len <= {7'd0, ec_len}; ec_pend <= 1'b0;
            end else if (tp_pend) begin                    // a line every 63 us: ahead of bulk slot reads
                busy <= 1'b1; typ <= UP_TAPE_AUD; len <= UP_TAPE_AUD_LEN; tp_lat <= tp_buf; tp_pend <= 1'b0;
            end else if (img_ready && !img_done) begin     // img_ready drops the cycle after img_done
                busy <= 1'b1; typ <= UP_IMG_DATA; len <= UP_IMG_DATA_LEN + img_d;
            end
        end else if (!nxt_v || take) begin
            if (!fetch) begin
                img_sel    <= p[4:0] - 5'd2;
                total      <= len + 16'd6;
                fetch      <= 1'b1;
            end else begin
                fetch <= 1'b0;
                nxt_v <= 1'b1;
                k     <= k + 1'b1;
                if (k == 16'd0) begin
                    nxt <= LINK_SYNC;
                    crc <= LINK_CRC_INIT;
                end else if (k == 16'd1) begin
                    nxt <= typ;
                    crc <= crc16(crc, typ);
                end else if (k == 16'd2) begin
                    nxt <= len[7:0];
                    crc <= crc16(crc, len[7:0]);
                end else if (k == 16'd3) begin
                    nxt <= len[15:8];
                    crc <= crc16(crc, len[15:8]);
                end else if (p < len) begin
                    nxt <= pay;
                    crc <= crc16(crc, pay);
                end else if (p == len) nxt <= crc[7:0];
                else nxt <= crc[15:8];
                if (k + 1'b1 == total) begin
                    busy    <= 1'b0;
                    packets <= packets + 1'b1;
                    if (typ == UP_IMG_DATA) img_done <= 1'b1;
                end
            end
        end
    end

    // Serializer: 8 clocks per byte.
    reg [2:0] c;
    reg       on;
    reg [7:0] cur;
    always @(posedge clk) begin
        take <= 1'b0;
        if (rst) begin
            on     <= 1'b0;
            strobe <= 1'b0;
            c      <= 3'd0;
        end else if (!on) begin
            if (nxt_v && !take) begin
                cur  <= nxt;
                nib  <= nxt[3:0];
                take <= 1'b1;
                on   <= 1'b1;
                c    <= 3'd1;
            end
        end else begin
            c <= c + 1'b1;
            case (c)
                3'd2: strobe <= 1'b1;
                3'd4: nib    <= cur[7:4];
                3'd6: strobe <= 1'b0;
                3'd7: on     <= 1'b0;
                default: ;
            endcase
        end
    end
endmodule
