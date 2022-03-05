import numpy as np
import plotly.graph_objects as go
import pickle

with open('estimates.txt', 'rb') as f_rd1:
    est = pickle.load(f_rd1)#np.loadtxt('estimates.txt')
with open('gt.txt', 'rb') as f_rd:
    gt = pickle.load(f_rd)#np.loadtxt('gt.txt')

errors = np.absolute(np.subtract(est, gt))
per = np.multiply(np.divide(errors, gt), 100)
print(est[:10], gt[:10], errors[:10])

x = gt
y = errors

counts_dict = {}
for i in range(len(gt)):
    key = str(gt[i])+'/'+str(errors[i])
    if key in counts_dict:
        counts_dict[key] += 1
    else:
        counts_dict[key] = 1

new_x = []
new_y = []
new_z = []
for key in counts_dict:
    key_gt, key_er = key.split('/')
    key_er = float(key_er)
    key_gt = float(key_gt)
    new_x.append(key_gt)
    new_y.append(key_er)
    new_z.append(counts_dict[key])
print(len(new_x))
new_x = np.array(new_x)
new_y = np.array(new_y)
new_z = np.array(new_z)
#exit()

#val=np.array(range(len(per)))
#counts=np.array(per)

print(np.amin(gt), np.amax(gt))


#val, counts = np.unique(per, return_counts=True)
#counts = y
print(len(errors))
fig = go.Figure()
fig.add_trace(go.Scatter(
    x=new_x, y=new_y, customdata=new_z, mode='markers', hovertemplate = 'gt: %{x}<br>'+'er: %{y}<br>'+'Frequency: %{customdata}<extra></extra>'))
fig.update_layout(title='error in distance estimation vs ground truth - in meters', xaxis_title='ground truth (mt)', yaxis_title='error (mt)', title_x=0.5)
fig.show()
