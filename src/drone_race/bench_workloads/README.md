Fixed CPU workloads for `drone-race bench`. Each file is a controller that hovers in place
and does a fixed amount of work in every `act()`, independent of the flight. `bench` runs them
on your computer and compares the median `act()` time with the same workloads measured on the
scoring server (`drone_race/data/reference_machine.json`) to scale the timing budget.
They are not examples of how to fly.
