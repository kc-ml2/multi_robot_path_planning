import array
import numpy as np

# PPM header
width = 1000
height = 1000
maxval = 255
ppm_header = f'P6 {width} {height} {maxval}\n'

# PPM image data (filled with blue)
image = array.array('B', [255, 255, 255] * width * height)

count = 0
total_size = width*height
percentage_cover = 0.3

while (count < percentage_cover * total_size):
	x = np.random.randint(40, width-40)
	y = np.random.randint(40, height-40)
	dx = np.random.randint(40, 100)
	dy = np.random.randint(40, 100)
	
	for i in range(x, x+dx):
		for j in range(y, y+dy):
			if j > height-40 or i > width-40:
				continue
			index = 3 * (j * width + i)
			image[index] = 0
			image[index + 1] = 0
			image[index + 2] = 0
			count += 1

# Save the PPM image as a binary file
with open('resources/ppm/blank.ppm', 'wb') as f:
	f.write(bytearray(ppm_header, 'ascii'))
	image.tofile(f)
