// Data-tape line bytes by cell class (cell width - 4 DAC samples): idx (2), data, CRC-16 (2).
function [7:0] dt_line_bytes(input [1:0] cls);
    case (cls)
        2'd0:    dt_line_bytes = 8'd21;
        2'd1:    dt_line_bytes = 8'd17;
        2'd2:    dt_line_bytes = 8'd14;
        default: dt_line_bytes = 8'd12;
    endcase
endfunction
function [4:0] dt_data_bytes(input [1:0] cls);
    dt_data_bytes = dt_line_bytes(cls) - 8'd4;
endfunction
