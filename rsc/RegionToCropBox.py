import csv

def get_max_point(point):
    return [(point[2]-point[0]),(point[3]-point[1]),0.3]

def get_translation(point):
    return [point[0],point[1],0.8]

def main():

    Regions = []
    i = 0

    with open('regions.csv', 'rb') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
        for row in spamreader:
            region = []
            for elt in row:
                region.append(float(elt))
            Regions.append(region)

    returnString = '<CropBoxList name="Map">'
    for t in Regions:

        returnString += '<CropBox name="Region_'+str(i)+'">'
        returnString += '<min_pt x="0" y="0" z="0"/>'
        max_point = get_max_point(t)
        returnString += '<max_pt x="'+str(max_point[0])+'" y="'+str(max_point[1])+'" z="'+str(max_point[2])+'"/>'
        returnString += '<rotation x="0" y="0" z="0"/>'
        translation = get_translation(t)
        returnString += ' <translation x="'+str(translation[0])+'" y="'+str(translation[1])+'" z="'+str(translation[2])+'"/>'
        returnString += '</CropBox>'
        i += 1

    returnString += '</CropBoxList>'

    f = open('./CropBoxList.xml', 'r+')
    f.write(returnString)
    f.close()

    print returnString

if __name__ == '__main__':
    main() 
