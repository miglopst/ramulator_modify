trace_length = 5
file_name = "hbm.trace"
#3bit, 0-7
channel_range =[0,1,2,3,4,5,6,7]
#1bit, 0-1
pseudo_channel_range = [0,1]
#2bit, 0-3
bank_group_range = [0,1,2,3]
#2bit, 0-3
bank_range = [0]
#14bit, 0-2^14-1
row_range = [0]
#5bit, 0-2^5-1
#column_range = [0,1,2,3]
column_range = 32
#5bit, 0-2^5-1
tx_range = [0]
#read or write
rw_range = ['R']

repeat_cnt = 1

def padhexa(s):
	return '0x' + s[2:].zfill(9)

f = open(file_name, "w")
for rp in range(0,repeat_cnt):
	for col in range(0,column_range):
		for ba in bank_range:
			for bg in bank_group_range:
				for pc in pseudo_channel_range:
					for ch in channel_range:
						for row in row_range:
							for tx in tx_range:
								for rw in rw_range:
									addr = '{0:b}'.format(row).zfill(14)+\
									'{0:b}'.format(ba).zfill(2)+'{0:b}'.format(bg).zfill(2)+\
									'{0:b}'.format(pc).zfill(1)+'{0:b}'.format(col).zfill(5)+'{0:b}'.format(ch).zfill(3)+'{0:b}'.format(tx).zfill(5)
									print addr
									addr = padhexa(hex(int(addr,2)))
									f.write(addr+' '+rw+'\n')
f.close()
