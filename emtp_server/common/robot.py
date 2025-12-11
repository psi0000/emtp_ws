actions = ["InspectObject <object>", "Delivery <object>"]
actions = ', '.join(actions)

# List of robots with different configurations 

# ALL SKILLS - INF MASS (UGV1,UGV2,UGV3,UGV4)
R1 = {'name': 'R1', 'type':'UGV',  'skills': ['InspectObject','Delivery'],    
        'capabilities': {
        'speed': 'fast',
        'mobility': 'cannot move between floors'
        },
    }

R2 = {'name': 'R2', 'type':'UGV',   'skills': ['InspectObject','Delivery'],    
        'capabilities': {
        'speed': 'fast',
        'mobility': 'cannot move between floors'
        },
    }

R3 = {'name': 'R3','type':'UAV',    'skills': ['InspectObject','Delivery'],    
         'capabilities': {
        'speed': 'fast',
        'mobility': 'can move between floors, and on the stairs'
        }, 
    }

R4 = {'name': 'R4','type':'UAV',    'skills': ['InspectObject','Delivery'],    
         'capabilities': {
        'speed': 'fast',
        'mobility': 'can move between floors, and on the stairs'
        }, 
    }
R5 = {'name': 'R5','type':'UAV',    'skills': ['InspectObject','Delivery'],    
         'capabilities': {
        'speed': 'fast',
        'mobility': 'can move between floors, and on the stairs'
        }, 
    }
R6 = {'name': 'R6','type':'UAV',    'skills': ['InspectObject','Delivery'],    
         'capabilities': {
        'speed': 'fast',
        'mobility': 'can move between floors, and on the stairs'
        }, 
    }
R7 = {'name': 'R7','type':'UAV',    'skills': ['InspectObject','Delivery'],    
         'capabilities': {
        'speed': 'fast',
        'mobility': 'can move between floors, and on the stairs'
        }, 
    }
R8 = {'name': 'R8','type':'UAV',    'skills': ['InspectObject','Delivery'],    
         'capabilities': {
        'speed': 'fast',
        'mobility': 'can move between floors, and on the stairs'
        }, 
    }
R9 = {'name': 'R9','type':'UAV',    'skills': ['InspectObject','Delivery'],    
         'capabilities': {
        'speed': 'fast',
        'mobility': 'can move between floors, and on the stairs'
        }, 
    }
R10 = {'name': 'R10','type':'UAV',    'skills': ['InspectObject','Delivery'],    
         'capabilities': {
        'speed': 'fast',
        'mobility': 'can move between floors, and on the stairs'
        }, 
    }
robots = [R1, R2, R3,R4,R5,R6,R7,R8,R9,R10]