from hardware_module import Field
import typing
from math import sqrt
from NHK2024_Camera_Library import Silo


class ScoreingSilos:
    def __init__(self, silos:typing.List[Silo]) -> None:
        self.opponent_ball_num = 0
        self.my_ball_num = 0
        self.scores = []
        self.is_silo_full = False
        self.silos = silos
    def get_score(self, silo:Silo) -> None:
        self.my_ball_num = silo.get_my_team_ball_cnt()
        self.opponent_ball_num = silo.get_opponent_team_cnt()
        self.pos = silo.get_position()
        # 距離 x[cm]*-1点
        # 計3個　-1000点
        # 相手2　3000点
        # 相手1　自分1　5000点
        # 自分2　4000点
        # 相手1　0
        # 自分1　1000
        # 0個　2000
        self.is_silo_full = False
        
        if (self.my_ball_num + self.opponent_ball_num) == 0:
            self.score = 3000
        elif self.my_ball_num == 1 and self.opponent_ball_num == 0:
            self.score = 2000
        elif self.my_ball_num == 0 and self.opponent_ball_num == 1:
            self.score = 1000
        elif self.my_ball_num == 2 and self.opponent_ball_num == 0:
            self.score = 5000
        elif self.my_ball_num == 1 and self.opponent_ball_num == 1:
            self.score = 6000
        elif self.my_ball_num == 0 and self.opponent_ball_num == 2:
            self.score = 4000
        elif (self.my_ball_num + self.opponent_ball_num) == 3:
            self.score = 0
            self.is_silo_full = True
        
        # 距離によるスコアの加算
        self.score -= sqrt(self.pos[0]*self.pos[0] + self.pos[1]*self.pos[1])/10

        return self.score
    
    def get_silo_full(self) -> bool:
        return self.is_silo_full
    
    def sort_by_score(self):
        for s in self.silos:
            self.scores.append(self.get_score(s))

        self.silos = [x for _, x in sorted(zip(self.scores, self.silos), key=lambda pair: pair[0], reverse=True)]
        return self.silos
