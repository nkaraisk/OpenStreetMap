# OpenStreetMap
To πρόγραμμα υλοποιεί ένα γράφο, με τη βοήθεια πραγματικών δεδομένων που θα λαμβάνουμε από αρχεία της πλατφόρμας OpenStreetMap(https://www.openstreetmap.org/). Για την υλοποίηση του γράφου και όλων των λειτουργιών του χρησιμοποιήθηκαν έτοιμες κλάσεις από τη βιβλιοθήκη STL.

Η λειτουργία του είναι η εξής:

-i <filepath>  :   Import Graph from <filepath>
-c             :   Compact Graph
-p <sid> <eid> :   Estimate the shortest path between start
                   node with <sid> and end node with <eid>
-b <sid>       :   Print bfs starting from node with <sid>
-d <sid>       :   Print dfs starting from node with <sid>
-q             :   Exit without memory leaks
