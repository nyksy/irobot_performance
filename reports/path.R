# The following code to create a dataframe and remove duplicated rows is always executed and acts as a preamble for your script: 

# dataset <- data.frame(time, x, y)
# dataset <- unique(dataset)

# Paste or type your script code here:

library("ggplot2")

ggplot() + geom_path(dataset, mapping=aes(x, y))