import rosbag
bag = rosbag.Bag('run.bag')
i = 0
for topic, msg, t in bag.read_messages(topics=['th', 'Xw', 'Yw']):
    print(f'topic: {topic}, msg: {msg}, time: {t}')
    i += 1

print(f'toatl msgs: {i}')
bag.close()