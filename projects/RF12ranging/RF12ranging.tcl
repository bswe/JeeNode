Jm doc "Driver for the HM3 power metering sketch."

proc process {device data} {
  # Called on each incoming message.
  variable configs
  dict extract $data message
  if {[regexp {^\[\S+\]\s\w i\S+ g(\d+) @ (\d+) MHz} $message - g m]} {
    Log RF12demo {config $device RF12-$m.$g}
    set configs($device) RF12-$m.$g
  } elseif {[string is list -strict $message] &&
            [lindex $message 0] eq "OK" &&
            [info exists configs($device)]} {
    set msg [lassign $message - hdr]
    set dev $configs($device).[expr {$hdr % 32}]
    submit $dev $msg [dict replace $data raw [binary format c* $msg]]
  }
}

Jm rev {$Id: RF12demo.tcl 7623 2011-04-27 00:52:48Z jcw $}
