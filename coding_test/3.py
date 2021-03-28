def fold(paper, i):
    return [(paper[i+n+1] if i+n+1 < len(paper) else 0) + (paper[i-n] if i-n >= 0 else 0)
            for n in range(max(len(paper)-i-1, i+1))]
    
def solution(paper, n):
    papers = [paper]
    answer = max(paper)
    for _ in range(n):
        new_papers = []
        for p in papers:
            for i in range(len(p)-1):
                folded = fold(p, i)
                max_folded = max(folded)
                if max_folded > answer: answer = max_folded
                new_papers.append(folded)
        papers = new_papers
    return answer