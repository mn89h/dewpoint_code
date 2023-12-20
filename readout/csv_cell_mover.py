import csv

def move_values(csv_file, firstRow, rowIncrease):
    iteration = 0
    
    while True:
        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            rows = list(reader)
            startRow = firstRow + iteration * rowIncrease
            if startRow > len(rows):
                break
            print(startRow)
            startCol = 2
            for rowI in range(startRow, len(rows)):
                for colI in range(startCol, len(rows[rowI])):
                    rows[rowI-1][colI] = rows[rowI][colI]
                    
        with open("output.csv", 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(rows[:-1])

        iteration = iteration + 1

# Replace 'input.csv' with the path to your CSV file

move_values('output.csv', 22, 19)