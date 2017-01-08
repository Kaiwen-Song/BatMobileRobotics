
# Request File Name and create the file reference
fileName = raw_input("Please enter a filename for a txt file: ")
filePointer = open(fileName + ".txt", 'r')

# Loop through the file createing the nested lists
coords = []
for currentLine in filePointer:
    coords.append(currentLine.split())

# Get the length of the set of coord and calculate the means for x and y
n = len(coords)
x_mean = sum(float(i[0]) for i in coords) / n
y_mean = sum(float(i[1]) for i in coords) / n

# Calculate the matrix
topLeft = sum( pow((float(i[0]) - x_mean), 2) for i in coords) / n
diagonal = sum( ((float(i[0]) - x_mean) * (float(i[1]) - y_mean)) for i in coords) / n
bottomRight = sum( pow((float(i[1]) - y_mean), 2) for i in coords) / n

result = [ [topLeft, diagonal], [diagonal, bottomRight]]
print("\nBelow is the resulting covariance matrix:\n")
for row in result:
    print row
