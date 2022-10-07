import datetime


def date_format(s, thread='', pre=''):
    print(f'{pre}[{datetime.datetime.now().strftime("%X.%f")[:-3]}{f"-{thread}" if thread else ""}] {s}')
