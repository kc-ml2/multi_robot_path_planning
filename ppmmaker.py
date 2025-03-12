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
percentage_cover = 0.5

while (count < percentage_cover * total_size):
	x = np.random.randint(0, width)
	y = np.random.randint(0, height)
	dx = np.random.randint(75, 100)
	dy = np.random.randint(75, 100)
	
	for i in range(x, x+dx):
		for j in range(y, y+dy):
			if j > width-1 or i > height-1:
				continue
			if j < 100 and i < 100:
				continue
			if j < 100 and i > 900:
				continue
			if j > 900 and i < 100:
				continue
			if j > 900 and i > 900:
				continue
			index = 3 * (j * width + i)
#			image[index] = 0
#			image[index + 1] = 0
#			image[index + 2] = 0
			count += 1


# Save the PPM image as a binary file
with open('resources/ppm/blank.ppm', 'wb') as f:
	f.write(bytearray(ppm_header, 'ascii'))
	image.tofile(f)
