from covaps.msg import Order
        
def create_order(type_, val_):
    order = Order()
    order.val = int(val_)
    order.type = type_
    return order
