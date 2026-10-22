# Any chipset source/probe instance; quartus_stp -t probe.tcl CABLE INSTANCE CMD..., each CMD
# "read [N]" (N lines of BITS, MSB first; default 1) or "src BITS" (writes the source bits; an x keeps a bit).
source [file join [file dirname [info script]] common.tcl]
set name [lindex $argv 1]
open_probes [lindex $argv 0] [list $name]
foreach c [lrange $argv 2 end] {
    if {[lindex $c 0] eq "read"} {
        set n [expr {[llength $c] > 1 ? [lindex $c 1] : 1}]
        for {set i 0} {$i < $n} {incr i} { puts "BITS [read_bits $name]" }
    } elseif {[lindex $c 0] eq "src"} {
        write_src $name [lindex $c 1]
    } else {
        puts stderr "unknown command: $c"
        exit 1
    }
}
end_insystem_source_probe
