import multiprocessing


def my_func(limit):
    for i in range(limit):
        print(i)


with multiprocessing.Pool(processes=2) as pool:
    params = [100, 100]
    results = pool.map(my_func, params)

