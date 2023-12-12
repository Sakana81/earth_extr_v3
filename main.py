import laspy
import numpy as np
import open3d as o3d
import argparse

def cmdParser():
    parser = argparse.ArgumentParser(description='Example of usage: \n'
                                                 'python main.py --lasfile=C:\..\..filename.las (str)'
                                                 '--radius=10 (int) --divider=7 (int) --lasname=file.las')

    parser.add_argument(
        '--lasfile',
        type=str,
        help='path to las file to extract earth points from'
    )
    parser.add_argument(
        '--radius',
        type=int,
        help='type = int \nradius in which earth points are selected (bigger radius - less points, '
             'smaller - more points are selected)'
    )

    parser.add_argument(
        '--step',
        type=int,
        help='type = int \noptional parameter to filter excess points above the ground'
    )
    parser.add_argument(
        '--lasname',
        type=int,
        help='type = str \nname of resulting file example: "file.las"'
    )
    my_namespace = parser.parse_args()
    return my_namespace.lasfile, my_namespace.radius, my_namespace.step, my_namespace.lasname


def read_las(path):
    las = laspy.read(path)
    header = las.header
    points = las.xyz
    ground = points[np.where(las.classification == 2)]

    points_sorted = points[np.lexsort((points[:, 0], points[:, 1], points[:, 2]))]
    x_min = las.header.x_min
    x_max = las.header.x_max
    y_min = las.header.y_min
    y_max = las.header.y_max
    return header, points_sorted, x_min, x_max, y_min, y_max, ground


def split(pointcloud, step, x_min, x_max, y_min, y_max):
    x_step = (x_max - x_min) / step
    y_step = (y_max - y_min) / step
    points = [[] for _ in range(step ** 2)]
    for dot in pointcloud:
        i = j = 0

        while (i * x_step) + x_min < dot[0]:
            i += 1
        while (j * y_step) + y_min < dot[1]:
            j += 1
        i -= 1
        j -= 1
        n = j * step + i
        points[n].append(dot)

    return points



def getFloor_v2(dots, radius):
    local_minima = []
    for i in range(dots.shape[0] - 1):
        mask = np.sqrt((dots[:, 0] - dots[i, 0]) ** 2 + (dots[:, 1] - dots[i, 1]) ** 2) <= radius
        if dots[i, 2] == np.min(dots[mask], axis=0)[2]:
            local_minima.append(tuple(dots[i]))
    return np.array(local_minima)

def getFloor_v3(dots):
    res = []


    return res

def plot_o3d(earth):
    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(earth)
    o3d.visualization.draw_geometries([geom])

def write_las(name, header, points):

    outfile = laspy.create(point_format=0)
    outfile.X = points[:,0]
    outfile.Y = points[:,1]
    outfile.Z = points[:,2]
    outfile.write(name)
    """
    header = laspy.header.LasHeader()
    ground_las = laspy.LasData(header)
    ground_las.points = points
    ground_las.write(name)
    """

def main():
    laspath, divider, radius, lasname = cmdParser()
    #laspath = 'C:\\Users\pickles\Downloads\Telegram Desktop\LYSVA_RGB_NIR_9\Lysva_may_PP9_D_G_O.las'
    #laspath = 'C:\\Users\pickles\Downloads\Lysva_12052022_VLS.las'


    #step = 100
    #k = 12
    step = divider
    k = radius

    header, points, x_min, x_max, y_min, y_max, ground = read_las(laspath)
    radius = (x_max - x_min) / (step * k)
    cells = split(points, step, x_min, x_max, y_min, y_max)
    #print('Done splitting')
    #print(f'{x_max-x_min}')
    res = []
    for i, cell in enumerate(cells):
        earth = getFloor_v2(np.array(cell), radius)
        res.extend(earth)
        #print(f'#{i} slice processed')
    #plot_o3d(ground)
    write_las(lasname, header, np.array(res))
    plot_o3d(res)


if __name__ == '__main__':
    main()
    #print(timeit.timeit("main()", setup="from __main__ import main"))
