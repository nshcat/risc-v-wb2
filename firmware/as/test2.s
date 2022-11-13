			lui t2, 0x0 					# Flag that gets flipped
		
			#lui t1, %hi(0x17D7840)			# Threshold to compare against
			#addi t1, t1, %lo(0x17D7840)

			lui t1, %hi(0x4)				# Threshold to compare against
			addi t1, t1, %lo(0x4)
		
			lui t0, 0x0						# Initialize counter

.loop:  	addi t0, t0, 0x1				# Increment counter
			bne t0, t1, .do_loop			# Continue if threshold not reached
			not t2, t2
			lui t0, 0x0
			
.do_loop:	j .loop
	