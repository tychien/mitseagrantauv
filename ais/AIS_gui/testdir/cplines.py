from csv import DictReader

class Copy():

    def __init__(self):
        path = "/home/tychien/Downloads/shipinformation_202109.csv"
        with open(path,'r') as read_obj:        
            csv_dict_reader = DictReader(read_obj)
            counter = 0
            for row in csv_dict_reader:
                counter += 1
                if counter <=30:
                    print(row['MMSI'],row['Record_Time'])
                else:
                    break
if __name__ == '__main__':
    copy = Copy()
