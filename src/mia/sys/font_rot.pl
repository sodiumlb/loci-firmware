my $x = 0;
my @ch;
while(<>){
	foreach my $line (split(",")){
		if( $line =~ m/0x(\w+)/){

			my $input = $1;
			#print "$input, ";
			my $bits = hex($input);
			$ch[0][$x] = ($bits & 0b00000001);
			$ch[1][$x] = ($bits & 0b00000010)>>1;
			$ch[2][$x] = ($bits & 0b00000100)>>2;
			$ch[3][$x] = ($bits & 0b00001000)>>3;
			$ch[4][$x] = ($bits & 0b00010000)>>4;
			$ch[5][$x] = ($bits & 0b00100000)>>5;
			$ch[6][$x] = ($bits & 0b01000000)>>6;
			$ch[7][$x] = ($bits & 0b10000000)>>7;

			$x = $x + 1;
			if($x == 8){
				print "\n\t ";
				for my $j (0 .. 7){
					my $tr_bits = 
						$ch[$j][0]<<0 | $ch[$j][1]<<1 | $ch[$j][2]<<2 | $ch[$j][3]<<3 | 
						$ch[$j][4]<<4 | $ch[$j][5]<<5 | $ch[$j][6]<<6 | $ch[$j][7]<<7;
					printf("0x%02x, ", $tr_bits);
				}
				$x=0;
			}
		}else{ print $line; }
	}
	
}	
