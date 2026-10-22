// The link's packet CRC, one byte at a time; include after link_proto.vh.
function [15:0] crc16(input [15:0] c, input [7:0] b);
    integer k;
    reg [15:0] x;
    begin
        x = c ^ {b, 8'd0};
        for (k = 0; k < 8; k = k + 1) x = x[15] ? {x[14:0], 1'b0} ^ LINK_CRC_POLY : {x[14:0], 1'b0};
        crc16 = x;
    end
endfunction
