

class Errors(TypedDict):


class State(TypedDict):
    error_loading_data: str 
    ETC ETC


def data_loader():
    state: State = {gar gar gar}
    while carry_on_running(state):
        raw_data_batch = read_batch(state)
        parsed_data = parse_data_batch(raw_data_batch)
        state = update_state(state, parsed_data)
        yield parsed_data


def main():
    neural_network = load_network() 
    neural_network.fit_generator(data_loader)


# while carry_on_running(state):
#     raw_input_data = do_IO(state2output(state))
#     state = update_state(state, parse_input_data(raw_input_data))
        

# def main():
#     err, used_data, data_batch = load_data_batch.main(used_data)
#     neural_network = train_net.main(neural_network, data_batch)
