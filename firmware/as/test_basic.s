			lui t0, %hi(0x4000)				# Systick tick register address
			addi t0, t0, %lo(0x4000)
		
			lui t1, %hi(0xFFFF)				
			addi t1, s1, %lo(0xFFFF)
				
			sw t1, 0(t0)
			
		
.loop:	j .loop
