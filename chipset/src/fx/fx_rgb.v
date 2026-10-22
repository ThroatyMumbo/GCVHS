// RGB pixel effects, combinational. Per channel: posterize (3 bits), then invert, then halve on odd
// output lines (scanlines). rotate then takes R, G, B from G, B, R.
module fx_rgb (
    input  wire       invert,
    input  wire       posterize,
    input  wire       scanline,
    input  wire       rotate,
    input  wire       odd_line,
    input  wire [7:0] r_in,
    input  wire [7:0] g_in,
    input  wire [7:0] b_in,
    output wire [7:0] r,
    output wire [7:0] g,
    output wire [7:0] b
);
    function [7:0] post(input [7:0] x);
        begin
            post = posterize ? {x[7:5], x[7:5], x[7:6]} : x;
            if (invert) post = ~post;
            if (scanline && odd_line) post = {1'b0, post[7:1]};
        end
    endfunction

    wire [7:0] r0 = post(r_in);
    wire [7:0] g0 = post(g_in);
    wire [7:0] b0 = post(b_in);
    assign {r, g, b} = rotate ? {g0, b0, r0} : {r0, g0, b0};
endmodule
