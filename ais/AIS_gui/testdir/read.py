import csv

class Read():
    
    
    def __init__(self):
        mydict  = []
        path = '/home/tychien/Downloads/shipinformation_202109.csv'
        with open(path,'r') as csvfile:
            reader = csv.reader(csvfile)
            dict_from_csv = {row[0]:row[1] for rows in reader}
            
            #for i in range(3):
            #    line   = csvfile.readline()
            #    print(line)
    

if __name__ == '__main__':
    read = Read()

