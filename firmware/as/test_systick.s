			lui t0, %hi(0x4020)				# Systick tick register address
			addi t0, t0, %lo(0x4020)
		
			lui s1, %hi(1000)				# Difference we want
			addi s1, s1, %lo(1000)
			
			lui s0, 0x0						# Value that we are going to negate/flip
			
			lw t1, 0(t0)					# First timestamp
			
.loop:		lw t2, 0(t0)					# Load systick value
			sub t2, t2, t1					# Calculate difference between timestamps
			blt t2, s1, .do_loop			# Continue looping if less than target value
			addi t1, t2, 0					# Update time stamp
			not s0, s0						# Negate register
		
.do_loop:	j .loop
