days_in_month = {1:31, 2:28, 3:31, 4:30, 5:31, 6:30, 7:31, 8:31, 9:30, 10:31, 11:30}

def get_days(date):
    days = 0
    y, m, d = date.split('/')
    for month in range(1, int(m)):
        days += days_in_month[month]
    days += int(d)
    return days

def get_grade(money):
    if money < 10000:
        return 0
    if money < 20000:
        return 1
    if money < 50000:
        return 2
    if money < 100000:
        return 3
    return 4

def solution(purchase):
    
    answer =  [0, 0, 0, 0, 0]
    history = [0]*365
    
    for pur in purchase:
        date, money = pur.split(' ')
        history[get_days(date)-1] += int(money)
    
    money_in_30days = 0

    for day in range(365):
        money_in_30days += history[day]
        money_in_30days -= history[day-30] if day >= 30 else 0
        answer[get_grade(money_in_30days)] += 1

    return answer