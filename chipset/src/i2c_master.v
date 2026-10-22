// Minimal single-master I2C: one register write or one register read per transaction.
// SCL is driven push-pull (sole master, no clock stretching on this bus); SDA is open-drain.
module i2c_master #(
    parameter integer QUARTER = 125     // clk cycles per quarter SCL period: 50 MHz / 500 = 100 kHz
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    input  wire       read,             // 0: write wdata to reg_addr, 1: read reg_addr into rdata
    input  wire [6:0] dev,
    input  wire [7:0] reg_addr,
    input  wire [7:0] wdata,
    output reg        busy,
    output reg        done,             // one-cycle pulse; nack/rdata valid with it
    output reg        nack,
    output reg  [7:0] rdata,
    output reg        scl,
    inout  wire       sda
);
    localparam [2:0] OP_START = 3'd0, OP_WBYTE = 3'd1, OP_RBYTE = 3'd2, OP_STOP = 3'd3, OP_END = 3'd4;

    reg        sda_low;
    reg  [1:0] sda_sync;
    reg        rd;
    reg  [6:0] dev_l;
    reg  [7:0] reg_l, dat_l;
    reg  [2:0] step;
    reg  [3:0] bitn;                    // 0-7 data bits, 8 = ack slot
    reg  [1:0] q;                       // quarter within the current bit
    reg  [7:0] div;
    reg  [2:0] op;
    reg  [7:0] wbyte;

    assign sda = sda_low ? 1'b0 : 1'bz;

    // Write: S dev+W reg data P.  Read: S dev+W reg Sr dev+R data NACK P.
    always @* begin
        wbyte = 8'h00;
        if (!rd)
            case (step)
                3'd0:    op = OP_START;
                3'd1:    begin op = OP_WBYTE; wbyte = {dev_l, 1'b0}; end
                3'd2:    begin op = OP_WBYTE; wbyte = reg_l; end
                3'd3:    begin op = OP_WBYTE; wbyte = dat_l; end
                3'd4:    op = OP_STOP;
                default: op = OP_END;
            endcase
        else
            case (step)
                3'd0:    op = OP_START;
                3'd1:    begin op = OP_WBYTE; wbyte = {dev_l, 1'b0}; end
                3'd2:    begin op = OP_WBYTE; wbyte = reg_l; end
                3'd3:    op = OP_START;
                3'd4:    begin op = OP_WBYTE; wbyte = {dev_l, 1'b1}; end
                3'd5:    op = OP_RBYTE;
                3'd6:    op = OP_STOP;
                default: op = OP_END;
            endcase
    end

    always @(posedge clk) begin
        sda_sync <= {sda_sync[0], sda};
        done     <= 1'b0;
        if (rst) begin
            busy    <= 1'b0;
            scl     <= 1'b1;
            sda_low <= 1'b0;
            nack    <= 1'b0;
            div     <= 8'd0;
        end else if (!busy) begin
            if (start) begin
                busy  <= 1'b1;
                rd    <= read;
                dev_l <= dev;
                reg_l <= reg_addr;
                dat_l <= wdata;
                step  <= 3'd0;
                bitn  <= 4'd0;
                q     <= 2'd0;
                div   <= 8'd0;
                nack  <= 1'b0;
            end
        end else if (div != QUARTER - 1) begin
            div <= div + 1'b1;
        end else begin
            div <= 8'd0;
            case (op)
                OP_START: case (q)      // also a repeated start: entered with SCL low
                    2'd0: sda_low <= 1'b0;
                    2'd1: scl     <= 1'b1;
                    2'd2: sda_low <= 1'b1;
                    2'd3: scl     <= 1'b0;
                endcase
                OP_STOP: case (q)
                    2'd0: sda_low <= 1'b1;
                    2'd1: scl     <= 1'b1;
                    2'd2: sda_low <= 1'b0;
                    2'd3: ;
                endcase
                OP_WBYTE: case (q)
                    2'd0: sda_low <= bitn == 4'd8 ? 1'b0 : ~wbyte[3'd7 - bitn[2:0]];
                    2'd1: scl     <= 1'b1;
                    2'd2: if (bitn == 4'd8 && sda_sync[1]) nack <= 1'b1;
                    2'd3: scl     <= 1'b0;
                endcase
                OP_RBYTE: case (q)      // SDA released throughout; the released ack slot is our NACK
                    2'd0: sda_low <= 1'b0;
                    2'd1: scl     <= 1'b1;
                    2'd2: if (bitn != 4'd8) rdata <= {rdata[6:0], sda_sync[1]};
                    2'd3: scl     <= 1'b0;
                endcase
                default: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            endcase
            q <= q + 1'b1;
            if (q == 2'd3) begin
                if ((op == OP_WBYTE || op == OP_RBYTE) && bitn != 4'd8)
                    bitn <= bitn + 1'b1;
                else begin
                    bitn <= 4'd0;
                    step <= step + 1'b1;
                end
            end
        end
    end
endmodule
