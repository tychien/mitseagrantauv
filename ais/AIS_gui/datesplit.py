from pathlib  import Path
from datetime import datetime
from csv import DictReader, DictWriter


def splitByDate(readpath):
    datelist = []
    MMSIlist = []
    dtformat = '%Y-%m-%d %H:%M:%S'
    with open(readpath, 'r') as read_obj:
        csv_dict_reader = DictReader(read_obj)
        for row in csv_dict_reader:
            MMSI = row['MMSI']
            
            record_time_d = datetime.strptime(row['Record_Time'],dtformat)
            year          = record_time_d.year
            month         = record_time_d.month
            day           = record_time_d.day
            date = str(year)+'{:02d}'.format(month)+'{:02d}'.format(day)
            if date in datelist:
                pass
            else:
                datelist.append(date)
            
            if MMSI in MMSIlist:
                pass
            else:
                MMSIlist.append(MMSI)
    print(datelist)
    print(MMSIlist)


if __name__ == '__main__':
    splitbydate = splitByDate(str(Path.home())+'/mitseagrantauv/ais/AIS_gui/CHe1/CHe1_1K.csv')
