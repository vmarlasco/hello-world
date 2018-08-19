# Arguments
In this file we will take notes about the reflexions and diferetn arguments that have shape the final solution.

## Classes
* The classes that has been selected are *extinguisher*, *door*, *emergency exit" because of thir possible applications in indoor
  environments (rescue, guidance, path planning) and because the are very common (what allows us to used as reference and improve
  map building), but there are not a large amount of them, what is perfect because of the limited resources of the raspberry pi:
  it is not neede to storage a lot of different objects, and between frames the processing is reduced.
  
## Performance
* We have to notice that object detection implies models with large size than object recognition (**search for comparison**), because the previous uses the later.

* Other detector systems like face recognitions give a response of 1 sencond, using OpenCV (more optimized) and detect one class


## Tracker
* Another "real time" tracking algorithmls implemented in raspberry py, are based the on simplifacations shuch as color substracting (ball examample), or scene difference [paper](http://cs231n.stanford.edu/reports/2017/pdfs/808.pdf)

## SqueezeDet
* Fewer parameters: smaller for the deployment of final application, and faster

## Future work
* Implement the NN in OpenCV, which is more optimized for RPi
