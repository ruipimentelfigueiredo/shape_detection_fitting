from PIL import Image
import random, os
import argparse
import numpy as np
grid_x=8
grid_y=8

image_size_x=400
image_size_y=300
margin_x=10
margin_y=10
count = 1


total_images=grid_x*grid_x
path='/home/rui/merged_dataset/train/images_clusters/cylinder'

image_name='cylinders_mosaic.pdf'

parser = argparse.ArgumentParser()
parser.add_argument('-p', 
                    '--path', 
                    default=str, 
                    #type=list, 
                    help="Path to fitting logs folder",
                    )
parser.add_argument('-i', 
                    '--image', 
                    default=image_name, 
                    type=str, 
                    help="Output image")
parser.add_argument('-gx', 
                    '--grid_x', 
                    default=grid_x, 
                    #type=list, 
                    help="Grid x number of images")
parser.add_argument('-gy', 
                    '--grid_y', 
                    default=grid_y, 
                    #type=list, 
                    help="Grid y number of images")
parser.add_argument('-margin_x', 
                    '--margin_x', 
                    default=margin_x, 
                    #type=list, 
                    help="x margin between images")
parser.add_argument('-margin_y', 
                    '--margin_y', 
                    default=margin_y, 
                    #type=list, 
                    help="y margin between images")

args=parser.parse_args()
path=args.path
image_name=args.image
grid_x=int(args.grid_x)
grid_y=int(args.grid_y)

margin_x=int(args.margin_x)
margin_y=int(args.margin_y)
new_im = Image.new('RGB', (image_size_x*grid_x,image_size_y*grid_y),(255, 255, 255))

random_filenames=[]
i=0
while i<total_images:
    random_folder = random.sample([x for x in os.listdir(path) if os.path.isdir(os.path.join(path, x))],1)
    files_path=path+'/'+str(random_folder[0])+'/'
    try:
        random_filename = random.sample([x for x in os.listdir(files_path) if os.path.isfile(os.path.join(files_path, x))],1)
    except ValueError:
        continue
    random_filenames.append(str(path+'/'+str(random_folder[0])+'/'+random_filename[0]))
    i+=1

crop_type='top'
index = 0
for i in xrange(int(margin_x*0.5),image_size_x*grid_x,image_size_x):
    for j in xrange(int(margin_y*0.5),image_size_y*grid_y,image_size_y):
        # If height is higher we resize vertically, if not we resize horizontally
        img = Image.open(random_filenames[index])
        # Get current and desired ratio for the images
        img_ratio = img.size[0] / float(img.size[1])
        ratio = image_size_x / float(image_size_y)
        #The image is scaled/cropped vertically or horizontally depending on the ratio
        if ratio > img_ratio:
            img = img.resize((image_size_x, image_size_x * image_size_y / image_size_x),Image.ANTIALIAS)
            # Crop in the top, middle or bottom
            if crop_type == 'top':
                box = (margin_x*0.5, margin_y*0.5, img.size[0]-margin_x*0.5, image_size_y-margin_y*0.5)
            elif crop_type == 'middle':
                box = (0, (img.size[1] - image_size_y) / 2, img.size[0], (img.size[1] + image_size_y) / 2)
            elif crop_type == 'bottom':
                box = (0, img.size[1] - image_size_y, img.size[0], img.size[1])
            else :
                raise ValueError('ERROR: invalid value for crop_type')
        else:
            img = img.resize((image_size_y * img.size[0] / img.size[1], image_size_y), Image.ANTIALIAS)
            # Crop in the top, middle or bottom
            if crop_type == 'top':
                box = (margin_x, margin_y, image_size_x-margin_x*0.5, img.size[1]-margin_y*0.5)
            elif crop_type == 'middle':
                box = ((img.size[0] - image_size_x) / 2, 0, (img.size[0] + image_size_x) / 2, img.size[1])
            elif crop_type == 'bottom':
                box = (img.size[0] - image_size_x, 0, img.size[0], img.size[1])
            else:
                raise ValueError('ERROR: invalid value for crop_type')
        box=(np.floor(box[0]).astype(int), np.floor(box[1]).astype(int), np.floor(box[2]).astype(int), np.floor(box[3]).astype(int))
        img = img.crop(box)

        new_im.paste(img, (i,j))
        index+=1

new_im.show()

new_im.save(image_name,format='pdf')
