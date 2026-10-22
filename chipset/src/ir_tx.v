// Panasonic (Kaseikyo) IR: 8T/4T header, 48 pulse-distance bits LSB first (1T mark, 1T or 3T space), stop mark,
// T = 432 us on a 1/3-duty carrier; a send repeats it REPS times, REPEAT_US start to start, like the remote.
module ir_tx #(
    parameter integer CLK_HZ     = 50_000_000,
    parameter integer CARRIER_HZ = 36_700,
    parameter integer T_US       = 432,
    parameter integer REPS       = 2,
    parameter integer REPEAT_US  = 130_000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        send,            // pulse; ignored while busy
    input  wire [47:0] payload,         // vendor in the low 16 bits
    output reg         tx,              // high lights the LED
    output reg         busy,
    output reg  [15:0] sent             // sends completed
);
    localparam integer CARRIER = CLK_HZ / CARRIER_HZ;
    localparam integer T_CLK   = CLK_HZ / 1_000_000 * T_US;
    localparam integer REP_CLK = CLK_HZ / 1_000_000 * REPEAT_US;

    reg [47:0] frame, data;
    reg [6:0]  k;                       // entry: even marks, odd spaces, 98 the stop mark
    reg [3:0]  units;                   // T left in the entry
    reg [$clog2(T_CLK)-1:0]   tcnt;
    reg [$clog2(CARRIER)-1:0] ccnt;
    reg [$clog2(REP_CLK)-1:0] fclk;
    reg [$clog2(REPS+1)-1:0] rep;
    reg        in_frame;

    wire [6:0] kn = k + 1'b1;

    always @(posedge clk) begin
        ccnt <= ccnt == CARRIER - 1 ? 1'b0 : ccnt + 1'b1;
        tx   <= in_frame && !k[0] && ccnt < CARRIER / 3;
        if (rst) begin
            busy     <= 1'b0;
            in_frame <= 1'b0;
            sent     <= 16'd0;
        end else if (!busy) begin
            if (send) begin
                busy     <= 1'b1;
                frame    <= payload;
                data     <= payload;
                rep      <= 1'b0;
                k        <= 7'd0;
                units    <= 4'd8;
                tcnt     <= 1'b0;
                ccnt     <= 1'b0;
                fclk     <= 1'b0;
                in_frame <= 1'b1;
            end
        end else begin
            fclk <= fclk + 1'b1;
            if (in_frame) begin
                if (tcnt != T_CLK - 1) tcnt <= tcnt + 1'b1;
                else begin
                    tcnt <= 1'b0;
                    if (units != 4'd1) units <= units - 1'b1;
                    else if (k == 7'd98) in_frame <= 1'b0;
                    else begin
                        k <= kn;
                        if (kn == 7'd1) units <= 4'd4;
                        else if (kn[0]) begin
                            units <= data[0] ? 4'd3 : 4'd1;
                            data  <= data >> 1;
                        end else begin
                            units <= 4'd1;
                            ccnt  <= 1'b0;
                        end
                    end
                end
            end else if (fclk == REP_CLK - 1) begin
                if (rep == REPS - 1) begin
                    busy <= 1'b0;
                    sent <= sent + 1'b1;
                end else begin
                    rep      <= rep + 1'b1;
                    data     <= frame;
                    k        <= 7'd0;
                    units    <= 4'd8;
                    tcnt     <= 1'b0;
                    ccnt     <= 1'b0;
                    fclk     <= 1'b0;
                    in_frame <= 1'b1;
                end
            end
        end
    end
endmodule
