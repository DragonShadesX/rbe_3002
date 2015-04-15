#!/usr/bin/env python


def transform_map_meters_to_grid_cells(cordinate, map_info):
    #x = (float(grid_x + .5) * map_info.resolution + float(map_info.origin.position.x))
    x_cord = (cordinate[0] - float(map_info.origin.position.x))/map_info.resolution #- .5
    #y = (float(grid_y + .5) * map_info.resolution + float(map_info.origin.position.y))
    y_cord = (cordinate[1] - float(map_info.origin.position.y))/map_info.resolution #- .5
    cell = (int(x_cord), int(y_cord))
    #print cell
    return cell

def transform_grid_cells_to_map_meters(cordinate, map_info):
    x_cord = (float(cordinate[0] + .5) * map_info.resolution + float(map_info.origin.position.x))
    y_cord = (float(cordinate[1] + .5) * map_info.resolution + float(map_info.origin.position.y))
    x_cord = round(x_cord, 3)
    y_cord = round(y_cord, 3)
    cell = (x_cord, y_cord)
    #print cell
    return cell
