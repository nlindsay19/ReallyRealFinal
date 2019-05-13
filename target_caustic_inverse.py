import sys
import numpy as np 
from numpy import fft
import math
from scipy import signal
from skimage import filters
from skimage import io, color, transform, img_as_ubyte, img_as_float
from matplotlib import pyplot
import random
from scipy import ndimage

#https://www.peterkovesi.com/papers/ai97.pdf
#https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1527851

imagePath = sys.argv[1]
maskPath = sys.argv[2]
mask = color.rgb2gray(img_as_float(io.imread(maskPath)))
image =  img_as_float(io.imread(imagePath))
image = color.rgb2gray(image)


for i in range(0, image.shape[0]):
	for j in range(0, image.shape[1]):
		if(mask[i,j] < 0.2):
			if(i % 2):
				image[i,j] = -1
			else:
				image[i,j] = -2
		
imageFFT = fft.fft2(image)

scales = 2 #2 #4
orientations = 20 #4 #6 

sigmaF = 0.65 #0.35 #0.65
wavelengthIncrement = 1.7# 3 #1.7
thetaStd = np.pi / orientations / 1.2

phaseSymmetry = np.zeros((image.shape[0], image.shape[1]))

symmetryTotal = np.zeros((image.shape[0], image.shape[1]))
amplitudeTotal = np.zeros((image.shape[0], image.shape[1]))
noiseAtOrientation = np.zeros((image.shape[0], image.shape[1]))
floor =  np.zeros((image.shape[0], image.shape[1]))

bank = []
for a in range(1,orientations + 1):
	centerAngle = (a - 1) * np.pi/orientations

	print(a)
	wavelength = 10

	symmetryAtOrientation = np.zeros((image.shape[0], image.shape[1]))
	amplitudeAtOrientation = np.zeros((image.shape[0], image.shape[1]))
	kernels = []
	for n in range(scales):
		kernel = np.zeros((image.shape[0], image.shape[1]))
		centerFrequency = 1/wavelength
		for i in range(kernel.shape[0]):
			for j in range(kernel.shape[1]):
				y = i - (kernel.shape[0]/2)
				x = j-(kernel.shape[1]/2)

				normalizedY = y / (kernel.shape[0]/2)
				normalizedX = x / (kernel.shape[1]/2)

				normalizedRadius = math.sqrt(normalizedY * normalizedY + normalizedX * normalizedX)

				elementRadial = np.exp(-1 * np.power( (normalizedRadius/ (centerFrequency / 0.5)),2) / (2 * (sigmaF) * (sigmaF)) )
				theta = math.atan2(-y,x)
				deltaSin = math.sin(theta) * math.cos(centerAngle) - math.cos(theta) * math.sin(centerAngle)
				deltaCosine = math.cos(theta) * math.cos(centerAngle) + math.sin(theta) * math.sin(centerAngle)

				deltaTheta = abs(math.atan2(deltaSin, deltaCosine))

				elementAngular = np.exp((-1 * deltaTheta * deltaTheta) / (2 * thetaStd * thetaStd))

				kernel[i, j] = elementAngular * elementRadial
				if(kernel[i,j] < 0.001):
					kernel[i,j] = 0
		kernel = fft.fftshift(kernel)
		kernels.append(kernel)
		wavelength = wavelength * wavelengthIncrement
	
	bank.append(kernels)


def calculatePhaseSymmetry(imageFFT, phaseSymmetry, symmetryTotal, amplitudeTotal):
	for a in range(1,orientations + 1):
		symmetryAtOrientation = np.zeros((image.shape[0], image.shape[1]))
		amplitudeAtOrientation = np.zeros((image.shape[0], image.shape[1]))
		for n in range(scales):		
			kernel = bank[a - 1][n]		
			convolved = imageFFT * kernel
			s1 = np.array(kernel.shape)
			s2 = np.array(imageFFT.shape)
			convolved = fft.ifft2(convolved)
			
			evens = np.real(convolved)
			odds = np.imag(convolved)
			amplitude = np.sqrt(np.power(evens, 2) + np.power(odds, 2)) 
			
			amplitudeAtOrientation += amplitude
			symmetryAtOrientation += np.maximum((np.abs(evens) - np.abs(odds))  , floor)
		
		amplitudeTotal += np.add(amplitudeAtOrientation, 0.00001)
		symmetryTotal += symmetryAtOrientation

	phaseSymmetry = np.divide(symmetryTotal , amplitudeTotal)
	phaseSymmetry[mask < 0.2] = 0.0
	return phaseSymmetry


def printImage(image, filename):
	saveImage = image.copy()
	saveImage[image < 0]= 0
	saveImage[image >1] = 1
	io.imsave(filename, img_as_ubyte(saveImage))


def maskMinMax(mask):
	minX = 10000
	minY = 100000
	maxX = 0
	maxY = 0
	for i in range(mask.shape[0]):
		for j in range(mask.shape[1]):
			if(mask[i,j] > 0.1):
				if i < minY:
					minY = i
				elif i > maxY:
					maxY = i

				if j < minX:
					minX = j 
				elif j > maxX:
					maxX = j 
	return minX, maxX, minY, maxY

def crop(image, mask, xmin, xmax, ymin, ymax):
	new = np.zeros((image.shape[0], image.shape[1]))
	ymid = math.floor((ymax + ymin)/2)
	xmid = math.floor((xmax + xmin)/ 2)
	radius = max((ymax - ymin)/2, (xmax - xmin)/2)
	for i in range(image.shape[0]):
		for j in range(image.shape[1]):			
			if(i > ymin and i < ymax and j > xmin and j < xmax):			
				new[i,j]= image[i, j]
	new[mask < 0.4] = 0
	return new

image = img_as_float(io.imread(imagePath))
image = color.rgb2gray(image)
for i in range(0, image.shape[0]):
	for j in range(0, image.shape[1]):
		if(mask[i,j] < 0.2):
			if(i % 2):
				image[i,j] = -1
			else:
				image[i,j] = -2
startImageFFT = fft.fft2(image) 
phaseSymmetryStart = calculatePhaseSymmetry(startImageFFT, phaseSymmetry, symmetryTotal, amplitudeTotal)

f = open("caustic_data.txt", "w")
for i in range(phaseSymmetryStart.shape[0]):
	for j in range(phaseSymmetryStart.shape[1]):
		num = phaseSymmetryStart[i,j]
		if(num < 0.001):
			num = 0
		f.write(str(num))
		f.write("\n")
f.close()
io.imsave("images/caustic.png", img_as_ubyte(phaseSymmetryStart))

