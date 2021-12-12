from csv import DictReader, DictWriter
import math 

def splitByRange(MIC_LAT, MIC_LON, readpath, writepath):
    counter = 0
    with open(readpath,'r') as read_obj:
        csv_dict_reader = DictReader(read_obj)
        for row in csv_dict_reader:
            lat = float(row['Latitude'])
            lon = float(row['Longitude'])
            if (lat != 'None' and lon != 'None'):
                R   = 6371000
                the1 = MIC_LAT  * math.pi/180
                the2 = lat      * math.pi/180
                thed = (lat-MIC_LAT) * math.pi/180
                lamdd= (lon-MIC_LON) * math.pi/180
                a = (math.sin(thed/2)* math.sin(thed/2))+(math.cos(the1) * math.cos(the2) * math.sin(lamdd/2)*math.sin(lamdd/2))
                c = 2 * math.atan2(math.sqrt(a),math.sqrt(1-a))
                d = R * c
                if d < 5000:
                    counter += 1 
                    print(counter, d, row['Latitude'], row['Longitude'],row['MMSI'])
                    with open(writepath, 'a') as wp:
                        fieldnames  = row.keys()
                        writer      = DictWriter(wp, fieldnames, delimiter=',')
                        if counter == 1:
                            writer.writeheader()
                        writer.writerow(row)

if __name__ == '__main__':
    splitbyran = splitByRange(24.04185,120.33571,'/home/tychien/mitseagrantauv/ais/AIS_gui/a.csv','/home/tychien/mitseagrantauv/ais/AIS_gui/CHe1_5K.csv')
