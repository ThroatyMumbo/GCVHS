# Data-tape slot dump (DTAP) for tools/datatape.py: quartus_stp -t dtap.tcl CABLE FIRST COUNT, one BITS line per slot,
# each read once the peek shows it. Reads and the loader reset go through probe.tcl DTAP.
source [file join [file dirname [info script]] common.tcl]
open_probes [lindex $argv 0] {DTAP}
set first [lindex $argv 1]
set count [lindex $argv 2]
for {set k $first} {$k < $first + $count} {incr k} {
    set want [format %015b $k]
    write_src DTAP "0$want"
    for {set tries 0} {$tries < 200} {incr tries} {
        set bits [read_bits DTAP]
        if {[string range $bits 0 14] eq $want} break
    }
    if {$tries == 200} { puts stderr "slot $k never showed"; break }
    puts "BITS $bits"
}
end_insystem_source_probe
