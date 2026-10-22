// One In-System Sources and Probes instance; the chipset tcl scripts find it by ID.
module jtag_probe #(
    parameter integer IDX = 0,
    parameter         ID  = "NONE",
    parameter integer PW  = 1,
    parameter integer SW  = 1
) (
    input  wire [PW-1:0] probe,
    output wire [SW-1:0] source
);
    altsource_probe #(
        .sld_auto_instance_index("YES"),
        .sld_instance_index(IDX),
        .instance_id(ID),
        .probe_width(PW),
        .source_width(SW),
        .enable_metastability("NO"),
        .lpm_type("altsource_probe")
    ) p (
        .probe(probe),
        .source(source)
    );
endmodule
