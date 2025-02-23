(define (domain warehouse_domain)
(:requirements :strips :typing :durative-actions :fluents :equality)

; Types
(:types
    robot
    ; shelve_waypoint - waypoint
    waypoint 
    box 
    zone 
)



; Predicates
(:predicates
    (robot_zone ?r - robot ?z - zone)
    (robot_at ?r - robot ?w - waypoint)
    (connected ?w1 - waypoint ?w2 - waypoint)
    (waypoint_from_zone ?w - waypoint ?z - zone)
    (box_at ?b - box ?w - waypoint)
    ; (box_of_type ?b - box ?z - zone)
    ; (shelve_at ?sw - shelve_waypoint ?z - zone)
    (idle_robot ?r - robot)
    (loaded_box ?b - box ?r - robot)
    
)
;  (:constants common_zone - waypoint)
(:functions
    (robot_capacity ?r -robot)
    (current_robot_load ?r -robot)
    ; (boxes_in_shelve ?sw - shelve_waypoint)
    ; (shelve_capacity ?sw - shelve_waypoint)

)


;posible mejora accion para el robot grande que obligue a que haya un mimimos de cajas en la zona de intercambio
; Actions
; move
; load_box
; unload_box

(:durative-action move
    :parameters (?r - robot ?from - waypoint ?to - waypoint ?z - zone)
    :duration ( = ?duration 10)
    :condition (and
        (at start (robot_at ?r ?from))
        (over all (connected ?from ?to))
        (over all (robot_zone ?r ?z))
        (over all (waypoint_from_zone ?from ?z))
        (over all (waypoint_from_zone ?to ?z))
        (at start (idle_robot ?r))
        ;idle_robot
        )
    :effect (and
        (at start (not(robot_at ?r ?from)))
        (at start (not(idle_robot ?r)))
        (at end (robot_at ?r ?to))
        (at end (idle_robot ?r))
    )
)

(:durative-action load_box
    :parameters (?r - robot ?b - box ?sw - waypoint ?z - zone)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (<= (+ (current_robot_load ?r) 1)(robot_capacity ?r))
        ; (> (boxes_in_shelve ?sw) 0)
        ))
        (at start (and 
        (idle_robot ?r);evitar coger mas de una caja a la vez )
        (box_at ?b ?sw)
        ))  
        (over all (and 
        
        ;;(shelve_at ?sw ?z)
        (robot_at ?r ?sw)
        
        ))
    )
    :effect (and 
        (at start (and 
        (not (idle_robot ?r))
        (not (box_at ?b ?sw))
        ))
        (at end (and 
        (idle_robot ?r)
        (loaded_box ?b ?r)
        ; (decrease (boxes_in_shelve ?sw) 1)
        (increase (current_robot_load ?r) 1)
        ))
    )
)


(:durative-action unload_box
    :parameters (?r - robot ?b - box ?sw - waypoint ?z - zone)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
        (> (current_robot_load ?r) 0)
        (idle_robot ?r)
        (loaded_box ?b ?r)
        ; (< (+ (boxes_in_shelve ?sw) 1)(shelve_capacity ?sw))

        ))
        (over all (and 
        (robot_at ?r ?sw)
        ;;(shelve_at ?sw ?z)
        ))
    )
    :effect (and 
        (at start (and 
        (not (idle_robot ?r))
        (not (loaded_box ?b ?r))
        ))
        (at end (and 
        (idle_robot ?r)
        (decrease (current_robot_load ?r) 1)
        ; (increase (boxes_in_shelve ?sw) 1)
        (box_at ?b ?sw)
        ))
    )
)

; (:durative-action load_box_from_common_zone; comentar con fran capacidades de common_zone
;     :parameters (?r - robot ?b - box ?z - zone)
;     :duration (= ?duration 1)
;     :condition (and 
;         (at start (and 
;         (< (+ (current_robot_load ?r) 1)(robot_capacity ?r))
;         (box_at ?b common_zone)
;         (idle_robot ?r)
        
;         ))
;         (over all (and 
;         (robot_at ?r common_zone)
;         (box_of_type ?b ?z)
;         (robot_zone ?r ?z)
;         ))
;     )
;     :effect (and 
;         (at start (and 
;         (not (idle_robot ?r))
;         (not (box_at ?b common_zone))
;         ))
;         (at end (and 
;         (idle_robot ?r)
;         (increase (current_robot_load ?r) 1)

;         ))
;     )
; )

; (:durative-action unload_box_at_common_zone
;     :parameters (?r - robot ?b - box ?z - zone)
;     :duration (= ?duration 1)
;     :condition (and 
;         (at start (and 
;         (> (current_robot_load ?r) 0)
;         (idle_robot ?r)
;         ))
;         (over all (and 
;         (robot_at ?r common_zone)
;         ))
;     )
;     :effect (and 
;         (at start (and 
;         (not (idle_robot ?r))

        
;         ))
;         (at end (and 
;         (idle_robot ?r)
;         (decrease (current_robot_load ?r) 1)
;         (box_at ?b common_zone)
;         ))
;     )
; )



; (:durative-action load_box_from_shelve 
;     :parameters (?r - robot ?b - box ?sw - shelve_waypoint ?z - zone)
;     :duration (= ?duration 1)
;     :condition (and 
;         (at start (and 
;         (< (+ (current_robot_load ?r) 1)(robot_capacity ?r))
;         (> (boxes_in_shelve ?sw) 0)
;         ))
;         (over all (and 
;         (box_at ?b ?sw)
;         ;;(shelve_at ?sw ?z)
;         (robot_at ?r ?sw)
;         (idle_robot ?r);evitar coger mas de una caja a la vez 
;         ))
;     )
;     :effect (and 
;         (at start (and 
;         (not (idle_robot ?r))
;         (not (box_at ?b ?sw))
;         ))
;         (at end (and 
;         (idle_robot ?r)
;         (decrease (boxes_in_shelve ?sw) 1)
;         (increase (current_robot_load ?r) 1)
;         ))
;     )
; )


; (:durative-action unload_box_at_shelve
;     :parameters (?r - robot ?b - box ?sw - shelve_waypoint ?z - zone)
;     :duration (= ?duration 1)
;     :condition (and 
;         (at start (and 
;         (> (current_robot_load ?r) 0)
;         (idle_robot ?r)
;         (< (+ (boxes_in_shelve ?sw) 1)(shelve_capacity ?sw))

;         ))
;         (over all (and 
;         (robot_at ?r ?sw)
;         ;;(shelve_at ?sw ?z)
;         ))
;     )
;     :effect (and 
;         (at start (and 
;         (not (idle_robot ?r))
;         ))
;         (at end (and 
;         (idle_robot ?r)
;         (decrease (current_robot_load ?r) 1)
;         (increase (boxes_in_shelve ?sw) 1)
;         (box_at ?b ?sw)
;         ))
;     )
; )

); End Domain