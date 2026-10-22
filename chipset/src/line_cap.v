// Grabs the 720 luma samples of one active line for JTAG (instance LCAP, tools/linecap.py). A change of
// arm captures line sel of the next field that has one; page p reads samples 32 p to 32 p + 31.
module line_cap (
    input  wire         clk,
    input  wire         sav,
    input  wire         y_en,
    input  wire [7:0]   y,
    input  wire [8:0]   line,
    input  wire         arm,            // toggle, synchronised
    input  wire [8:0]   sel,
    input  wire [4:0]   page,
    output reg  [7:0]   caps = 8'd0,    // captures completed
    output reg  [4:0]   page_out,       // page_data holds this page
    output reg  [255:0] page_data       // sample 32 page_out + i in bits 8 i + 7 .. 8 i
);
    reg [7:0] ram [0:1023];
    reg       arm_d, armed = 1'b0, active = 1'b0;
    reg [9:0] n;
    always @(posedge clk) begin
        arm_d <= arm;
        if (arm != arm_d) armed <= 1'b1;
        if (sav && armed && line == sel) begin
            active <= 1'b1;
            armed  <= 1'b0;
            n      <= 10'd0;
        end else if (active && n == 10'd720) begin
            active <= 1'b0;
            caps   <= caps + 1'b1;
        end else if (active && y_en) begin
            ram[n] <= y;
            n      <= n + 1'b1;
        end
    end

    reg [4:0] pg;
    reg [5:0] rd = 6'd33;
    reg [7:0] q;
    always @(posedge clk) begin
        q <= ram[{pg, rd[4:0]}];
        if (page != pg) begin
            pg <= page;
            rd <= 6'd0;
        end else if (rd != 6'd33) begin
            rd <= rd + 1'b1;
            if (rd != 6'd0) page_data[8 * (rd - 1) +: 8] <= q;
            if (rd == 6'd32) page_out <= pg;
        end
    end
endmodule
