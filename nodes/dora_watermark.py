import pickle


def load(inputs, key):
    data = pickle.loads(inputs[key])
    return data


def dump(data):
    pickled_data = pickle.dumps(data)
    return pickled_data
