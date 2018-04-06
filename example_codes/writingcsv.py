import csv
p={}
p[0,0]=0
p[1:1]=5
p[1,2]=2
p[2,0]=3
p[1,0]=4
with open('dict.csv', 'w') as f:  # This creates the file object for the context 
                                  # below it and closes the file automatically
    l = []
    for k, v in p.iteritems(): # Iterate over items returning key, value tuples
        l.append('%s: %s' % (str(k), str(v))) # Build a nice list of strings
    f.write(', '.join(l))                     # Join that list of strings and write out

with open('dict.csv', 'r') as f: # Again temporary file for reading
    d = {}
    l = f.read().split(',')      # Split using commas
    for i in l:
        values = i.split(': ')   # Split using ': '
        d[values[0]] = values[1] # Any type conversion will need to happen here
print d[1,1]

