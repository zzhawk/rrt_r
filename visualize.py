import json
import math
import matplotlib.pyplot as plt
import numpy as np

f = open("env.json", "r")
data = f.read()
env = json.loads(data)

play_x = [env["play"]["x"]["min"], env["play"]["x"]["min"], env["play"]["x"]["max"], env["play"]["x"]["max"], env["play"]["x"]["min"]]
play_y = [env["play"]["y"]["min"], env["play"]["y"]["max"], env["play"]["y"]["max"], env["play"]["y"]["min"], env["play"]["y"]["min"]]
plt.plot(np.asarray(play_x), np.asarray(play_y), "black")

def plot_circle(x, y, size, color="-b"):
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)

for ob in env["obstacle"]:
    plot_circle(float(env["obstacle"][ob]["x"]), float(env["obstacle"][ob]["y"]), float(env["obstacle"][ob]["radius"]))

plot_circle(env["start"]["x"], env["start"]["y"], 0.1, "red")
plot_circle(env["goal"]["x"], env["goal"]["y"], 0.1, "red")


for tr in env["tries"]:
    x = [env["tries"][tr]["start"]["x"],env["tries"][tr]["end"]["x"]]
    y = [env["tries"][tr]["start"]["y"],env["tries"][tr]["end"]["y"]]
    plt.plot(np.asarray(x), np.asarray(y))

if "path" in env:
    path_x = []
    path_y = []
    for i in range(len(env["path"])):
        path_x.append(env["path"][str(i)]["start"]["x"]) 
        path_y.append(env["path"][str(i)]["start"]["y"]) 
        plt.plot(np.asarray(path_x), np.asarray(path_y), linewidth=10.0, alpha=0.05, color="red")

plt.legend()

plt.show()
f.close()