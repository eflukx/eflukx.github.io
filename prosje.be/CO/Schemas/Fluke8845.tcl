#!/usr/bin/wish

# Fluke8845
# Voer een serie geautomatiseerde metingen uit met de Fluke8845
# De besturing gebeurt over het LAN
# Pros 2008

wm title . "Fluke8845"

proc do_it {} {
    global DeBeschrijving AantalSamples SampleGap MetingType anfil digfil

    # Verzin een unieke naam aan de hand van datum en tijd
    set FileName [exec date +%Y%m%d%H%M%S]
    set Datum [exec date]
    set InfoName $FileName
    append InfoName ".info"
    # Schrijf gegevens over de meting weg
    set fd [open "/pros/DigiScoopSamples/Fluke8845/$InfoName" w]
    puts $fd $Datum
    puts $fd " $DeBeschrijving"
    puts $fd $AantalSamples
    puts $fd $SampleGap
    switch $MetingType {
	1 {set conf "CONF:VOLT:DC\r\n"; puts $fd "Volt DC"}
	2 {set conf "CONF:VOLT:DC\r\n"; puts $fd "mVolt DC"}
	3 {set conf "CONF:VOLT:AC\r\n"; puts $fd "Volt AC"}
	4 {set conf "CONF:VOLT:AC\r\n"; puts $fd "mVolt AC"}
	5 {set conf "CONF:CURR:DC\r\n"; puts $fd "Ampere DC"}
	6 {set conf "CONF:CURR:AC\r\n"; puts $fd "Ampere AC"}
	7 {set conf "CONF:FREQ\r\n"; puts $fd "Frequentie"}
    }

    # Probeer een verbinding met de Fluke te maken
    for {set i 0} {$i < 5} {incr i} {
	set try [catch {set sockChan [socket 192.168.1.8 3490]} error]
	if {$try == 0} {
	    break
	}
	after 500
    }

    if {$try > 0} {
	puts stderr "Could not connect"
	puts stderr $error
	destroy .
    }
    fconfigure $sockChan -translation binary
    puts -nonewline $sockChan "*IDN?\r\n"
    flush $sockChan
    gets $sockChan line
    puts stderr "Connected with $line"
    puts -nonewline $sockChan "*CLS\r\n"
    flush $sockChan
    after 300
    puts -nonewline $sockChan "SYST:REM\r\n" ;# Schakel de Fluke in remote-bediening
    flush $sockChan
    after 300
    puts -nonewline $sockChan $conf
    flush $sockChan
    after 300
    puts -nonewline $sockChan "TRIG:DEL 0\r\n"
    flush $sockChan
    after 300
    if {$anfil == 1} {
	puts -nonewline $sockChan "FILT:DC:STAT ON\r\n"
	flush $sockChan
	puts $fd "AFIL=1"
	after 300
    } else {
	puts $fd "AFIL=0"
    }
    if {$digfil == 1} {
	puts -nonewline $sockChan "FILT:DC:DIG:STAT ON\r\n"
	flush $sockChan
	puts $fd "DFIL=1"
	after 300
    } else {
	puts $fd "DFIL=0"
    }
    puts -nonewline $sockChan "SAMP:COUN 1\r\n"
    flush $sockChan
    after 300
    puts -nonewline $sockChan "INIT\r\n"
    flush $sockChan
    after 300
    # Voor de x-as
    set x 0
    # Maximum y-as
    set y 0

    close $fd
    set fd [open "/pros/DigiScoopSamples/Fluke8845/$FileName" w]

    for {set i 0} {$i < 5} {incr i} {
	puts -nonewline $sockChan "READ?\r\n"
	flush $sockChan
	gets $sockChan line
	after 300
    }
    
    while {$AantalSamples > 0} {
	set AantalSamples [expr	$AantalSamples -1]
	update
	after $SampleGap
	puts -nonewline $sockChan "READ?\r\n"
	flush $sockChan
	gets $sockChan line
	scan $line "%f" v
	if {$MetingType == 2} {
	    set v [expr $v * 1000]
	} elseif {$MetingType == 3} {
	    set v [expr $v * 1000]
	}
	puts $fd [format "%02d\t%f" $x $v]
	flush $fd
	incr x
	if {$v > $y} {
	    set y $v
	}
    }
    close $fd

    after 500
    puts -nonewline $sockChan "syst:loc\r\n"
    flush $sockChan
    close $sockChan

    exec /pros/bin/FlukePlot $FileName

    destroy .
}

set w .entry
set c .frame.c
wm geometry . +200+100
set font -*-Fixed-Medium-R-*-*-*-130-75-75-*-*-ISO8859-1
set MetingType 1

frame .btn -relief groove -borderwidth 4
pack  .btn -side bottom -expand y -fill x -pady 2m
radiobutton .btn.a -text "Volt DC" -relief flat -variable MetingType -value 1
radiobutton .btn.b -text "mVolt DC" -relief flat -variable MetingType -value 2
radiobutton .btn.c -text "Volt AC" -relief flat -variable MetingType -value 3
radiobutton .btn.d -text "mVolt AC" -relief flat -variable MetingType -value 4
radiobutton .btn.e -text "Ampere DC" -relief flat -variable MetingType -value 5
radiobutton .btn.f -text "Ampere AC" -relief flat -variable MetingType -value 6
radiobutton .btn.g -text "Frequency" -relief flat -variable MetingType -value 7
pack .btn.a .btn.b .btn.c .btn.d .btn.e .btn.f .btn.g -side left -expand 1
button .btn.dismiss -text "GO!" -command "do_it"
pack .btn.dismiss -side left -expand 1

frame .beschrijving -relief groove -borderwidth 4
pack  .beschrijving -side top -expand y -fill x -pady 2m
label .beschrijving.la -font $font -text "Beschrijving meting: "
entry .beschrijving.tekstveld -relief sunken -textvariable DeBeschrijving -width 64
bind .beschrijving.tekstveld <Return> {
    focus .val.tekstveld
}
pack .beschrijving.la -side left
pack .beschrijving.tekstveld -side right

frame .val -relief groove -borderwidth 4
pack  .val -side top -expand y -fill x -pady 2m
label .val.la -font $font -text "Aantal sampels: "
entry .val.tekstveld -relief sunken -textvariable AantalSamples -width 8
bind .val.tekstveld <Return> {
    focus .val.tekstveldb
}

label .val.lab -font $font -text "Periode tussen samples in mSec: "
entry .val.tekstveldb -relief sunken -textvariable SampleGap -width 8
bind .val.tekstveldb <Return> {
    do_it
}
pack .val.la .val.tekstveld .val.lab .val.tekstveldb -side left

frame .fil -relief groove -borderwidth 4
pack .fil -side top -expand y -fill x -pady 2m
checkbutton .fil.analoog -text "Analoge filter on" -variable anfil
checkbutton .fil.digitaal -text "Digitale filter on" -variable digfil
pack .fil.analoog .fil.digitaal -side left
set anfil 1
set digfil 1
set AantalSamples 2400
set SampleGap 1000


focus .beschrijving.tekstveld
