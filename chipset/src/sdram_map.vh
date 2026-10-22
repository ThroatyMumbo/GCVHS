// SDRAM bank 0 row of a stored video line. Bit 8 is never set, so rows with it are free.
function [12:0] video_row(input [1:0] slot, input [7:0] line);
    video_row = {2'd0, slot, 1'b0, line};
endfunction
function [1:0] video_row_slot(input [12:0] row);
    video_row_slot = row[10:9];
endfunction
localparam [12:0] SPARE_ROW = 13'h100;
