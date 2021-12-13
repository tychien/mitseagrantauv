from pathlib import Path
from csv import DictReader, DictWriter
from datetime import datetime
def countShip(readpath):
    datelist = []
    datedict = {}
    mmsilist = []
    shiplist = []
    dtformat = '%Y-%m-%d %H:%M:%S'
    count0914 = 0
    with open(readpath,'r') as read_obj:
        csv_dict_reader = DictReader(read_obj)
        for row in csv_dict_reader:
            shiplist.append(row)
            mmsi= row['MMSI']
            record_time_d = datetime.strptime(row['Record_Time'],dtformat)
            year    = record_time_d.year
            month   = record_time_d.month
            day     = record_time_d.day
            date    = str(year)+'{:02d}'.format(month)+'{:02d}'.format(day)
            if date in datelist:
                pass
            else:
                datelist.append(date)

            if mmsi in mmsilist:
                pass
            else:
                mmsilist.append(mmsi)


        datedict = {key:[] for key in datelist}

        for ship in shiplist:
            record_time_b = datetime.strptime(ship['Record_Time'],dtformat)
            year    = record_time_b.year
            month   = record_time_b.month
            day     = record_time_b.day
            date    = str(year)+'{:02d}'.format(month)+'{:02d}'.format(day)
            datedict[date].append(ship)

        for date in datedict:
            for i in range(len(datedict[date])+1):
                print(datedict[date][i-1]['Record_Time'])



    #print((datedict['20210915'][19]['Record_Time']))
    print(len(shiplist))

if __name__ == '__main__':
    countshipt = countShip(str(Path.home())+'/mitseagrantauv/ais/AIS_gui/CHe1_5K.csv')
