# IR over one JTAG session (IRTX, IRRX) for tools/ir.py: quartus_stp -t irctl.tcl CABLE FIFO, a command per line:
# send P48 -> SENT, RX frames pages; tx P48 -> SENT ms; log MS -> S ms VSTA TAGW LTCP PTAG VITC lines, DONE; quit.
# quartus_stp -t irctl.tcl CABLE watch: READY idle L seq S, then FRAME pages (or TORN seq) per new frame, until killed.
source [file join [file dirname [info script]] common.tcl]
set logged {VSTA TAGW LTCP PTAG VITC}
set watch [expr {[lindex $argv 1] eq "watch"}]
open_probes [lindex $argv 0] [expr {$watch ? {IRRX} : [concat IRRX IRTX $logged]}]

proc rx_page {p} {
    write_src IRRX [format %04b $p]
    return [read_bits IRRX]
}
proc seq_of {bits} { scan [string range $bits 11 26] %b s; return $s }

# Page 0 is b; the rest of the frame's pages after it.
proc rx_frame {b} {
    scan [string range $b 2 10] %b len
    set pages [list $b]
    for {set p 1} {$p * 16 < $len} {incr p} { lappend pages [rx_page $p] }
    return $pages
}

proc ir_send {payload} {
    set tog [expr {1 - [string index [read_bits IRTX] 0]}]
    write_src IRTX "$tog$payload"
}

if {$watch} {
    set b [rx_page 0]
    set last [seq_of $b]
    say "READY idle [string index $b 0] seq $last"
    while 1 {
        set b [rx_page 0]
        set seq [seq_of $b]
        if {$seq != $last} {
            set pages [rx_frame $b]
            if {[seq_of [rx_page 0]] == $seq} { say "FRAME [join $pages]" } else { say "TORN $seq" }
            set last $seq
        }
        after 10
    }
}

set cmds [open [lindex $argv 1] {RDWR NONBLOCK}]    ;# our own write end: no EOF before the writer comes
say "READY"
while 1 {
    if {[gets $cmds line] < 0} {
        after 5
        continue
    }
    if {$line eq "quit"} break
    if {[lindex $line 0] eq "send"} {
        set seq0 [seq_of [rx_page 0]]
        ir_send [lindex $line 1]
        say "SENT"
        after 300
        set b [rx_page 0]
        set n [expr {([seq_of $b] - $seq0) & 0xFFFF}]
        say "RX $n [join [rx_frame $b]]"
    } elseif {[lindex $line 0] eq "tx"} {
        ir_send [lindex $line 1]
        say "SENT [clock milliseconds]"
    } elseif {[lindex $line 0] eq "log"} {
        set end [expr {[clock milliseconds] + [lindex $line 1]}]
        while {[set now [clock milliseconds]] < $end} {
            set s "S $now"
            foreach n $logged { append s " " [read_bits $n] }
            puts $s
        }
        say "DONE"
    } elseif {$line ne ""} {
        say "unknown command: $line"
    }
}
end_insystem_source_probe
