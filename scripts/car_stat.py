import os,sys

path = "log_res/log_res_emp"
namelist = open('caseNameList.txt','r')
nametext = namelist.readlines()
res = open('stat.csv','w')
safe = open('safe.txt','w')
unsafe = open('unsafe.txt', 'w')
safeNum = 0
unsafeNum = 0
time = 0.0
for case_name in nametext:
  name = case_name.rstrip()[:-4]
  f = open(path + '/' + name+'.res')
  text = f.read()
  if len(text) == 0:
    res.write(name + ',unknow\n')
  else:
    logfile = open(path + '/' + name+'.log')
    loglines = logfile.readlines()
    if len(loglines) == 0:
      logtime = 0
    else:
      logtime = loglines[-1].split(' ')[-2].strip()
    time = time + float(logtime)
    if text[0] == '1':
      res.write( name + ',unsafe,'+str(logtime)+'\n')
      unsafe.write(name+'\n')
      unsafeNum = unsafeNum+1
    elif text[0] == '0':
      res.write(name + ',safe,'+str(logtime)+'\n')
      safe.write(name + '\n')
      safeNum = safeNum + 1
    else:
      pass
print(" unsafe: " + str(unsafeNum) + " safe: " + str(safeNum) +" time: "+str(time))
