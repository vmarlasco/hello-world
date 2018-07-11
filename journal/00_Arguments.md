# Arguments
In this file we will take notes about the reflexions and diferetn arguments that have shape the final solution.

## Classes
* The classes that has been selected are *extinguisher*, *door*, *emergency exit" because of thir possible applications in indoor
  environments (rescue, guidance, path planning) and because the are very common (what allows us to used as reference and improve
  map building), but there are not a large amount of them, what is perfect because of the limited resources of the raspberry pi:
  it is not neede to storage a lot of different objects, and between frames the processing is reduced.
  
## Performance
* We have to notice that object detection implies models with large size than object recognition (**search for comparison**), because the previous uses the later.


## Tracker
* Another "real time" tracking algorithmls implemented in raspberry py, are based the on simplifacations shuch as color substracting (ball examample), or scene difference [paper](http://cs231n.stanford.edu/reports/2017/pdfs/808.pdf)
