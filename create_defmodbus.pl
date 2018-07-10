#!/usr/bin/perl

use strict;

open(IN, "modbus.cpp");

my $offs = 0;
while(<IN>) {
  chomp;
  if (/^\{[^,]+,\s*([0-9])\s*,[^,]+,[^,]+,\s*\/\/\s*([UIF]):(.*)$/) {
    # print $1. " " . $2 . " " . $3 . "\n";
    my $size;
    if ($1 eq '1') {
      $size = "8";
    } elsif ($1 eq '2') {
      $size = "16";
    } else {
      $size = "32";
    }
    my $type;
    if ($2 eq 'F') {
      $type = "float";
    } elsif ($2 eq 'I') {
      $type = "int" . $size . "_t";
    } else {
      $type = "uint" . $size . "_t";
    }
    print "// Type: $type\n";
    print "#define $3 $offs\n";
    $offs += 2;
  }
}

close(IN);
