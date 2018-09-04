# shapes 2018

This dataset contains the images with several household objects, which belong to one of 3 categories: cylinder, box or sphere.
Each image is annotated with bounding boxes and respective class labels.

<p align="center"> 
    <img src=.image/real_4.png>
</p>

--------------------

### Dataset Description

The dataset is composed of a total of 250 images, split into two sets of distinct objects.
The first set we use for training (`train`) and validation (`eval`) which have 175 and 26 images respectively.
The second set we use for testing (`test`) and has 49 images.

Images were recorded with a Kinect V2 sensor, and are stored as Full-HD (1920 x 1080) `JPEG` files.

Each image has a corresponding `XML` ground truth annotation, in the style of the [PASCAL VOC][pascal_voc] dataset.

### Processed data

Concretly regarding [our work][arxiv] we provide the processed data in the `processed` folder.
We include a condensed `.csv` with the preprocessed annotations for each dataset partition.
Even though we downscaled the images to 960 x 540, they are not provided as they are easy to resize locally.
However, we do provide the binary [Tensor Flow][tensor flow] record (`.record`) files which were used as input to [SSD][ssd], after image resize and annotation preprocessing.

### Citation

If you use this dataset in your work, please cite the following arXiv preprint:

```
TODO
```

### Basic statistics

This dataset is unbalanced with respect to number of class instances.
The number of objects per class is shown in the following table:

| Partition | # Box | # Cylinder | # Sphere | Total |
|-----------|-------|------------|----------|-------|
| train     | 564   | 244        | 241      | 1049  |
| eval      | 67    | 30         | 34       | 131   |
| test      | 106   | 104        | 53       | 263   |

After preprocessing the annotations (e.g. removing objects too small to be detected by SSD after resize) we obtained:

| Partition | # Box | # Cylinder | # Sphere | Total |
|-----------|-------|------------|----------|-------|
| train     | 502   | 209        | 86       | 797   |
| eval      | 64    | 28         | 18       | 110   |
| test      | 106   | 104        | 53       | 263   |

[arxiv]: Link
[pascal_voc]: http://host.robots.ox.ac.uk/pascal/VOC/voc2012/devkit_doc.pdf
[ssd]: https://arxiv.org/abs/1512.02325
[tensor flow]: https://www.tensorflow.org/
