def argsort(scores):
    scores_with_id = [(score, id) for id, score in enumerate(scores)]
    scores_with_id.sort(reverse=True)
    return [id for score, id in scores_with_id]
    
def solution(math_scores, english_scores):
    answer = 0
    math_sorted = argsort(math_scores)
    for rank, id1 in enumerate(math_sorted[:-1]):
        for id2 in math_sorted[rank+1:]:
            if english_scores[id1] > english_scores[id2]:
                answer += 1
    return answer