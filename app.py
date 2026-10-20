import heapq
import itertools
import streamlit as st

st.set_page_config(page_title='PathFinder X', page_icon='🧭', layout='wide')
st.title('🧭 PathFinder X')
st.caption('A* 최단 경로 탐색과 판단 과정 설명')

ROWS, COLS = 8, 10
START, GOAL = (3, 0), (3, 9)
DEFAULT_WALLS = {(r, 4) for r in range(1, 7)} | {(1, 7), (2, 7), (4, 7), (5, 7)}

def heuristic(pos):
    return abs(pos[0] - GOAL[0]) + abs(pos[1] - GOAL[1])

def solve(walls):
    counter = itertools.count()
    queue = [(heuristic(START), next(counter), START, 0)]
    best = {START: 0}
    parent = {}
    closed = set()
    logs = []
    while queue:
        f, _, node, g = heapq.heappop(queue)
        if node in closed or g != best[node]:
            continue
        h = heuristic(node)
        closed.add(node)
        candidates = sorted(
            [(p, cost, heuristic(p), cost + heuristic(p)) for p, cost in best.items() if p not in closed],
            key=lambda x: (x[3], x[2], x[0]),
        )
        logs.append({'node': node, 'g': g, 'h': h, 'f': f, 'visited': set(closed), 'candidates': candidates[:6]})
        if node == GOAL:
            path = [node]
            while path[-1] != START:
                path.append(parent[path[-1]])
            return path[::-1], logs
        for dr, dc in [(0, 1), (1, 0), (-1, 0), (0, -1)]:
            nxt = (node[0] + dr, node[1] + dc)
            if not (0 <= nxt[0] < ROWS and 0 <= nxt[1] < COLS):
                continue
            if nxt in walls or nxt in closed:
                continue
            ng = g + 1
            if ng < best.get(nxt, float('inf')):
                best[nxt] = ng
                parent[nxt] = node
                heapq.heappush(queue, (ng + heuristic(nxt), next(counter), nxt, ng))
    return [], logs

if 'walls' not in st.session_state:
    st.session_state.walls = set(DEFAULT_WALLS)

with st.sidebar:
    st.header('⚙️ 지도 설정')
    if st.button('기본 장애물 복원'):
        st.session_state.walls = set(DEFAULT_WALLS)
    if st.button('장애물 모두 지우기'):
        st.session_state.walls = set()
    st.info('지도의 빈 칸을 클릭하면 장애물을 추가하거나 제거할 수 있어요. S와 G는 고정되어 있습니다.')

st.subheader('① 장애물 지도 편집')
for r in range(ROWS):
    cols = st.columns(COLS, gap='small')
    for c in range(COLS):
        p = (r, c)
        if p == START:
            label = '🟢'
        elif p == GOAL:
            label = '🔴'
        elif p in st.session_state.walls:
            label = '⬛'
        else:
            label = '⬜'
        if cols[c].button(label, key=f'cell_{r}_{c}', use_container_width=True, disabled=p in (START, GOAL)):
            if p in st.session_state.walls:
                st.session_state.walls.remove(p)
            else:
                st.session_state.walls.add(p)
            st.rerun()

path, logs = solve(st.session_state.walls)
st.subheader('② A* 탐색 결과')
if path:
    st.success(f'최단 경로 발견! 이동 거리: {len(path)-1}칸 / 확장한 노드: {len(logs)}개')
else:
    st.error(f'목적지에 도달할 수 없습니다. 확장한 노드: {len(logs)}개')

st.markdown('**경로 지도** (🟦 최종 경로)')
for r in range(ROWS):
    line = []
    for c in range(COLS):
        p = (r, c)
        line.append('🟢' if p == START else '🔴' if p == GOAL else '⬛' if p in st.session_state.walls else '🟦' if p in path else '⬜')
    st.markdown(''.join(line))

st.subheader('③ 탐색 과정과 판단 근거')
st.caption('한 단계는 후보 칸 하나를 실제로 선택해 확장하는 과정입니다.')
index = st.slider('탐색 단계', 1, len(logs), 1)
log = logs[index - 1]
node = log['node']
st.write(f'**{index}단계:** {node} 위치를 선택했습니다.')
st.write(f"출발점부터의 비용 g = **{log['g']}**, 목표까지의 예상 비용 h = **{log['h']}**, 합계 f = **{log['f']}**")
st.info('A*는 탐색 대기 후보 중 f=g+h 값이 가장 작은 칸을 우선 선택합니다. 동점이면 구현에서 정한 순서로 처리합니다. 여기서 h는 상하좌우 이동의 맨해튼 거리입니다.')
if log['candidates']:
    st.write('**현재 칸을 확장한 뒤 남은 주요 후보**')
    st.dataframe([{'위치': str(p), 'g': g, 'h': h, 'f': f} for p, g, h, f in log['candidates']], hide_index=True)
else:
    st.write('현재 단계 이후 대기 중인 후보가 없습니다.')

st.markdown('**지금까지 확장한 칸 (🟨), 현재 선택한 칸 (🟧)**')
for r in range(ROWS):
    line = []
    for c in range(COLS):
        p = (r, c)
        line.append('🟢' if p == START else '🔴' if p == GOAL else '⬛' if p in st.session_state.walls else '🟧' if p == node else '🟨' if p in log['visited'] else '⬜')
    st.markdown(''.join(line))

st.subheader('④ 탐구 정리')
st.write('**탐구 질문:** 왜 A*는 모든 칸을 확인하지 않고도 최단 경로를 찾을 수 있을까?')
st.write('이 프로그램은 한 칸당 이동 비용을 1로 두고 상하좌우로만 이동합니다. 맨해튼 거리 휴리스틱은 실제 남은 이동 비용을 과대평가하지 않으므로, 이 조건에서 A*가 찾은 경로는 최단 경로입니다.')
st.caption('참고: 탐색 노드 수는 장애물 배치와 동점 처리 순서에 따라 달라질 수 있습니다.')
