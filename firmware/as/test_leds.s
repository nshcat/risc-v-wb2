			lui t0, %hi(0x4020)				# Systick tick register address
			addi t0, t0, %lo(0x4020)
		
			lui s1, %hi(1000)				# Difference we want
			addi s1, s1, %lo(1000)
				
			lw t1, 0(t0)					# First timestamp
			
			lui s3, 0
			
			
.loop:		lw t2, 0(t0)					# Load systick value
			sub s4, t2, t1					# Calculate difference between timestamps
			blt s4, s1, .do_loop			# Continue looping if less than target value
			addi t1, t2, 0					# Update time stamp
			
			lui s2, %hi(0x4000)				# LED register address
			addi s2, s2, %lo(0x4000)
			
			not s3, s3
			sw s3, 0(s2)
			
		
.do_loop:	j .loop
