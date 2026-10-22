# Shared by the chipset JTAG scripts (quartus_stp -t): open_probes finds the board and starts a session.

# The DE1's FPGA on the cable matching CABLE; idx(NAME) is each instance's index, and every one of NAMES must exist.
proc open_probes {cable names} {
    global idx
    set hw ""
    foreach h [get_hardware_names] { if {[string match "*$cable*" $h]} { set hw $h } }
    if {$hw eq ""} { puts stderr "no JTAG cable matching $cable"; exit 1 }
    set dev ""
    foreach d [get_device_names -hardware_name $hw] { if {[string match "*5CSE*" $d]} { set dev $d } }
    if {$dev eq ""} { puts stderr "no Cyclone V FPGA on $hw"; exit 1 }
    array set idx {}
    foreach inst [get_insystem_source_probe_instance_info -hardware_name $hw -device_name $dev] {
        set idx([lindex $inst 3]) [lindex $inst 0]
    }
    foreach n $names {
        if {![info exists idx($n)]} { puts stderr "probe $n not found; is the chipset programmed?"; exit 1 }
    }
    start_insystem_source_probe -hardware_name $hw -device_name $dev
}

# Source bits, MSB first; an x keeps that bit as it is.
proc write_src {name bits} {
    global idx
    if {[string first x $bits] >= 0} {
        set cur [read_source_data -instance_index $idx($name)]
        set merged ""
        for {set i 0} {$i < [string length $bits]} {incr i} {
            set ch [string index $bits $i]
            append merged [expr {$ch eq "x" ? [string index $cur $i] : $ch}]
        }
        set bits $merged
    }
    write_source_data -instance_index $idx($name) -value $bits
}

proc read_bits {name} {
    global idx
    return [read_probe_data -instance_index $idx($name)]
}

proc say {s} { puts $s; flush stdout; update }
