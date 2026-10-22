# Line capture and VITC diagnostics (LCAP, VDIA) for tools/linecap.py: quartus_stp -t lcap.tcl CABLE CMD...
# cap LINE -> CAP, then PAGE p BITS for pages 0-22 of that active line; vdia prints VDIA; vclr clears it.
source [file join [file dirname [info script]] common.tcl]
open_probes [lindex $argv 0] {LCAP VDIA}

proc bin {v w} { return [format %0${w}b $v] }
proc caps {} { scan [string range [read_bits LCAP] 0 7] %b c; return $c }

foreach c [lrange $argv 1 end] {
    set op [lindex $c 0]
    if {$op eq "cap"} {
        set line [lindex $c 1]
        set arm [string index [read_source_data -instance_index $idx(LCAP)] 0]
        set arm [expr {1 - $arm}]
        set c0 [caps]
        write_src LCAP "$arm[bin $line 9][bin 31 5]"
        set t0 [clock milliseconds]
        while {[caps] == $c0 && [clock milliseconds] - $t0 < 2000} { after 2 }
        if {[caps] == $c0} { say "CAP $line TIMEOUT"; continue }
        puts "CAP $line [expr {[clock milliseconds] - $t0}]"
        for {set p 0} {$p < 23} {incr p} {
            write_src LCAP "$arm[bin $line 9][bin $p 5]"
            set b [read_bits LCAP]
            while {[string range $b 8 12] ne [bin $p 5]} { set b [read_bits LCAP] }
            puts "PAGE $p [string range $b 13 end]"
        }
        flush stdout
    } elseif {$op eq "vdia"} {
        say "VDIA [read_bits VDIA]"
    } elseif {$op eq "vclr"} {
        write_src VDIA 1
        write_src VDIA 0
    } else {
        puts stderr "unknown command: $c"
    }
}
end_insystem_source_probe
