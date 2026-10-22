// IR remote in (raw mark/space timings, JTAG instance IRRX, source {page}) and Panasonic frames out for the VCR
// (JTAG instance IRTX: source {toggle, payload[47:0]}, a change of toggle sends; probe {toggle, busy, sends}).
module ir_top (
    input  wire        clk,             // CLOCK_50
    input  wire        rst,
    input  wire        rx,
    output wire        tx,
    input  wire        cpu_send,        // the game CPU's IR_HI write: a toggle
    input  wire [47:0] cpu_payload,
    output wire [15:0] rx_seq
);
    wire [3:0]   ir_page;
    wire [255:0] ir_data;
    wire [8:0]   ir_len;
    wire [31:0]  ir_t;
    wire         ir_over, ir_idle;
    ir_rx irrx (
        .clk(clk), .rst(rst), .rx(rx), .page(ir_page), .page_data(ir_data),
        .seq(rx_seq), .t_start(ir_t), .len(ir_len), .over(ir_over), .idle_level(ir_idle)
    );
    jtag_probe #(.IDX(3), .ID("IRRX"), .PW(319), .SW(4)) irp (
        .probe({ir_idle, ir_over, ir_len, rx_seq, ir_t, ir_page, ir_data}),
        .source(ir_page)
    );

    wire [48:0] irtx_src;
    reg  [2:0]  irtx_tog;
    wire        irtx_busy;
    wire [15:0] irtx_sent;
    // Whichever sent last, JTAG or the game CPU, supplies the payload.
    reg  [1:0]  cpu_ir_tog;
    reg         ir_from_cpu, ir_go;
    always @(posedge clk) begin
        irtx_tog   <= {irtx_tog[1:0], irtx_src[48]};
        cpu_ir_tog <= {cpu_ir_tog[0], cpu_send};
        ir_go      <= irtx_tog[2] != irtx_tog[1] || cpu_ir_tog[1] != cpu_ir_tog[0];
        if (cpu_ir_tog[1] != cpu_ir_tog[0]) ir_from_cpu <= 1'b1;
        else if (irtx_tog[2] != irtx_tog[1]) ir_from_cpu <= 1'b0;
    end
    ir_tx irtx (
        .clk(clk), .rst(rst), .send(ir_go), .payload(ir_from_cpu ? cpu_payload : irtx_src[47:0]),
        .tx(tx), .busy(irtx_busy), .sent(irtx_sent)
    );
    jtag_probe #(.IDX(4), .ID("IRTX"), .PW(18), .SW(49)) itp (
        .probe({irtx_src[48], irtx_busy, irtx_sent}),
        .source(irtx_src)
    );
endmodule
