import random

def fully_connected_graph_generator_to_csv(n, r, p):
    with open(f"fcgraph{n}.csv", "w") as f:
        f.write(str(n)+"\n")
        for i in range(1, n):
            for j in range(i+1, n+1):
                rnjesus = random.uniform(0,1)
                if rnjesus < p:
                    f.write(str(i) + "," + str(j) + "," + str(random.randint(r[0], r[1])) + "\n")


def main():
    n = int(input("Enter number of nodes: "))
    r = [int(x) for x in input("Enter range of edge weights: ").split()]
    p = float(input('Enter probability value to include an edge:'))
    fully_connected_graph_generator_to_csv(n, r, p)

if __name__ == "__main__":
    main()